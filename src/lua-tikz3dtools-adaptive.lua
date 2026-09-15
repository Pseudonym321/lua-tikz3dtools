--- Adaptive sampling for parametric curves and surfaces.
---
--- This module is deliberately numerical.  It estimates the geometric error
--- of the current piecewise-linear approximation by probing each parametric
--- element at a fixed nested stencil.  Surface samples are retriangulated in
--- the normalized parameter domain with an incremental Delaunay triangulation.
--- Curves retain parameter order and split only the intervals which need it.
local Adaptive = {}
Adaptive.__index = Adaptive

local Recovery = require "lua-tikz3dtools-recovery"
local Vector

function Adaptive:_set_classes(vclass)
    Vector = vclass
end

local huge = math.huge
local abs = math.abs
local sqrt = math.sqrt
local floor = math.floor
local max = math.max
local min = math.min

-- Internal numerical policy.  Adaptive surface topology lives in normalized
-- parameter space, so these are dimensionless relative tolerances rather than
-- drawing-coordinate tolerances.  Keep them centralized so predicate policy is
-- reviewable and does not drift across individual helpers.
local NUMERIC_POLICY = {
    orientation_relative = 1e-13,
    cocircular_relative = 1e-13,
    collinear_relative = 1e-11,
    interior_parameter = 1e-10,
    minimum_squared_scale = 1e-30,
    transition_floor = 1e-14,
}

-- Domain reconstruction policy is intentionally internal: these values define
-- how a finished curvature mesh is sewn to a numerically discovered validity
-- contour, not public surface semantics.
local DOMAIN_POLICY = {
    simplify_tolerance_divisor = 8,
    transition_refinement_divisor = 64,
    transition_max_iterations = 64,
    scout_min_steps = 4,
    scout_max_steps = 16,
    scout_feature_multiple = 4,
    collar_start_feature_multiple = 4,
    collar_target_spacing_fraction = 0.75,
    collar_initial_cell_fraction = 0.8,
    collar_max_layers = 8,
    constraint_recovery_passes = 4,
    constraint_recursion_limit = 32,
    constraint_flip_factor = 2,
    constraint_min_flips = 32,
}

local function finite_number(x)
    return type(x) == "number" and x == x and x ~= huge and x ~= -huge
end

local function table_key_count(value)
    local count = 0
    for _ in pairs(value) do count = count + 1 end
    return count
end

local function finite_vector(v)
    if not v or getmetatable(v) ~= Vector then
        return false
    end
    for i = 1, #v do
        if not finite_number(v[i]) then
            return false
        end
    end
    return true
end

local function point_distance(a, b)
    if not (finite_vector(a) and finite_vector(b)) then
        return huge
    end
    return a:_hdistance(b)
end

local function lerp_point(a, b, t)
    local out = {}
    local n = min(#a, #b)
    for i = 1, n - 1 do
        out[i] = a[i] + (b[i] - a[i]) * t
    end
    out[n] = a[n]
    return Vector:_new(out)
end

local function bary_point(a, b, c, wa, wb, wc)
    local out = {}
    local n = min(#a, #b, #c)
    for i = 1, n - 1 do
        out[i] = wa * a[i] + wb * b[i] + wc * c[i]
    end
    out[n] = a[n]
    return Vector:_new(out)
end

local function axis_value(start_value, stop_value, t)
    if t <= 0 then return start_value end
    if t >= 1 then return stop_value end
    return start_value + (stop_value - start_value) * t
end

local function clamp01(x)
    if x < 0 then return 0 end
    if x > 1 then return 1 end
    return x
end

local function number_key(x)
    -- 17 significant digits round-trip IEEE doubles and make evaluation cache
    -- keys stable without quantizing the caller's parameter domain.
    return ("%.17g"):format(x)
end

local function point_key(x, y)
    return number_key(x) .. "," .. number_key(y)
end

local function normalize_spec(spec, kind, initial_count)
    if type(spec) ~= "table" then
        spec = {}
    end

    local tolerance = spec.tolerance
    if not (finite_number(tolerance) and tolerance > 0) then
        tolerance = 0.01
    end

    local feature_tolerance = spec.feature_tolerance
    if not (finite_number(feature_tolerance) and feature_tolerance > 0) then
        feature_tolerance = 0.001
    end

    local max_depth = spec.max_depth
    if not (finite_number(max_depth) and max_depth >= 0 and max_depth == floor(max_depth)) then
        max_depth = 12
    end

    local budget_name = kind == "surface" and "max_triangles" or "max_segments"
    local budget_default = kind == "surface" and 1000 or 500
    local budget = spec[budget_name]
    if not (finite_number(budget) and budget >= initial_count and budget == floor(budget)) then
        budget = math.max(budget_default, initial_count)
    end

    return {
        tolerance = tolerance,
        feature_tolerance = feature_tolerance,
        max_depth = max_depth,
        budget = budget,
    }
end

-- ===========================================================================
-- Adaptive curves
-- ===========================================================================

-- Seven interior samples keep the estimator systematic rather than relying on
-- one privileged midpoint.  The set is nested under binary subdivision.
local curve_probe_fractions = {
    1/8, 1/4, 3/8, 1/2, 5/8, 3/4, 7/8,
}

local function curve_segment_score(segment, evaluate)
    local a, b = segment.a, segment.b
    local best_error = -1
    local best_t = 0.5
    local unsafe = not (a.valid and b.valid)

    for _, local_t in ipairs(curve_probe_fractions) do
        local t = a.t + (b.t - a.t) * local_t
        local probe = evaluate(t)
        if not probe.valid then
            unsafe = true
            best_error = huge
            best_t = t
            break
        elseif a.valid and b.valid then
            local chord = lerp_point(a.metric, b.metric, local_t)
            local err = point_distance(probe.metric, chord)
            if err > best_error then
                best_error = err
                best_t = t
            end
        end
    end

    if best_error < 0 then best_error = huge end
    segment.error = best_error
    segment.split_t = best_t
    segment.unsafe = unsafe
end

function Adaptive.curve(f, ustart, ustop, usamples, user_spec, metric)
    assert(Vector ~= nil, "Adaptive classes have not been initialized")
    assert(usamples >= 2, "adaptive curve requires at least two initial samples")

    local initial_segments = usamples - 1
    local spec = normalize_spec(user_spec, "curve", initial_segments)
    local cache = {}

    local function evaluate(t)
        t = clamp01(t)
        local key = number_key(t)
        local cached = cache[key]
        if cached then return cached end
        local u = axis_value(ustart, ustop, t)
        local ok, p = pcall(f, u)
        if not ok then
            Recovery.log_once("adaptive-curve-sample", "adaptive curve sample omitted", p)
            p = nil
        end
        local measured
        if finite_vector(p) then
            if metric then
                local metric_ok, metric_value = pcall(metric, p)
                if metric_ok then
                    measured = metric_value
                else
                    Recovery.log_once(
                        "adaptive-curve-metric",
                        "adaptive curve metric sample omitted",
                        metric_value
                    )
                end
            else
                measured = p
            end
        end
        local node = {
            t = t,
            u = u,
            point = p,
            metric = measured,
            valid = finite_vector(p) and finite_vector(measured),
        }
        cache[key] = node
        return node
    end

    local segments = {}
    for i = 0, usamples - 2 do
        local t0 = i / (usamples - 1)
        local t1 = (i + 1) / (usamples - 1)
        local segment = {
            a = evaluate(t0),
            b = evaluate(t1),
            depth = 0,
        }
        curve_segment_score(segment, evaluate)
        segments[#segments + 1] = segment
    end

    local exhausted = false
    while #segments < spec.budget do
        local worst_index
        local worst_priority = -1

        for i, segment in ipairs(segments) do
            local width = segment.b.t - segment.a.t
            -- feature_tolerance is only a localization scale for unsafe /
            -- non-finite behavior.  Smooth finite geometry is governed by the
            -- geometric tolerance and max_depth, so a large feature_tolerance
            -- never punches holes in an otherwise valid curve.
            local refinable = segment.depth < spec.max_depth
                and (not segment.unsafe or width > spec.feature_tolerance)
            local needs_refinement = segment.unsafe or segment.error > spec.tolerance
            if refinable and needs_refinement then
                local priority = segment.unsafe and huge or segment.error
                if priority > worst_priority then
                    worst_priority = priority
                    worst_index = i
                end
            end
        end

        if not worst_index then break end

        local segment = segments[worst_index]
        local split_t = segment.split_t
        local width = segment.b.t - segment.a.t
        if split_t <= segment.a.t + width * 1e-9
            or split_t >= segment.b.t - width * 1e-9
        then
            split_t = 0.5 * (segment.a.t + segment.b.t)
        end

        local mid = evaluate(split_t)
        local left = {
            a = segment.a,
            b = mid,
            depth = segment.depth + 1,
        }
        local right = {
            a = mid,
            b = segment.b,
            depth = segment.depth + 1,
        }
        curve_segment_score(left, evaluate)
        curve_segment_score(right, evaluate)

        segments[worst_index] = left
        table.insert(segments, worst_index + 1, right)
    end

    -- Diagnose whether the requested tolerance was unattainable inside the
    -- explicit budget/depth/localization limits.  Rendering still uses the
    -- best mesh obtained; invalid segments are omitted by the caller.
    local worst_error = 0
    local unresolved = 0
    local unmet = 0
    for _, segment in ipairs(segments) do
        if segment.unsafe then
            unresolved = unresolved + 1
        else
            if segment.error > worst_error then worst_error = segment.error end
            if segment.error > spec.tolerance then unmet = unmet + 1 end
        end
        if (segment.unsafe or segment.error > spec.tolerance)
            and #segments >= spec.budget
        then
            exhausted = true
        end
    end

    return segments, {
        segments = #segments,
        worst_error = worst_error,
        unresolved = unresolved,
        unmet = unmet,
        exhausted = exhausted,
        tolerance = spec.tolerance,
    }
end

-- ===========================================================================
-- Incremental 2D Delaunay triangulation in normalized (u,v) space
-- ===========================================================================

local function orient2d(a, b, c)
    return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x)
end

local function squared_distance2(a, b)
    local dx = b.x - a.x
    local dy = b.y - a.y
    return dx * dx + dy * dy
end

local function triangle_scale2(a, b, c)
    return max(
        squared_distance2(a, b),
        squared_distance2(b, c),
        squared_distance2(c, a),
        NUMERIC_POLICY.minimum_squared_scale
    )
end

local function orientation_epsilon(a, b, c, relative)
    return (relative or NUMERIC_POLICY.orientation_relative)
        * triangle_scale2(a, b, c)
end

local function make_triangle(a, b, c, points)
    local pa, pb, pc = points[a], points[b], points[c]
    local area = orient2d(pa, pb, pc)
    if abs(area) <= orientation_epsilon(pa, pb, pc) then
        return nil
    end
    if area < 0 then b, c = c, b end
    return {a, b, c}
end

local function incircle_contains(a, b, c, p)
    local ax, ay = a.x - p.x, a.y - p.y
    local bx, by = b.x - p.x, b.y - p.y
    local cx, cy = c.x - p.x, c.y - p.y

    local det = (ax * ax + ay * ay) * (bx * cy - cx * by)
        - (bx * bx + by * by) * (ax * cy - cx * ay)
        + (cx * cx + cy * cy) * (ax * by - bx * ay)

    local orientation = orient2d(a, b, c)
    local scale2 = max(
        ax * ax + ay * ay,
        bx * bx + by * by,
        cx * cx + cy * cy,
        NUMERIC_POLICY.minimum_squared_scale
    )
    local eps = NUMERIC_POLICY.cocircular_relative * scale2 * scale2

    -- Treat numerically cocircular points as outside the cavity.  Rectangular
    -- initial grids contain many exact cocircularities; a strict test avoids
    -- swallowing an arbitrary large cavity and leaves those ties to the fixed
    -- insertion order / containing-triangle fallback.
    if orientation > 0 then
        return det > eps
    end
    return det < -eps
end

local function point_in_triangle(p, a, b, c)
    local o1 = orient2d(a, b, p)
    local o2 = orient2d(b, c, p)
    local o3 = orient2d(c, a, p)
    local e1 = orientation_epsilon(a, b, p)
    local e2 = orientation_epsilon(b, c, p)
    local e3 = orientation_epsilon(c, a, p)
    local has_neg = o1 < -e1 or o2 < -e2 or o3 < -e3
    local has_pos = o1 > e1 or o2 > e2 or o3 > e3
    return not (has_neg and has_pos)
end

local function edge_key(a, b)
    if a < b then return a .. ":" .. b end
    return b .. ":" .. a
end

local function delaunay_insert(points, triangles, point_index)
    local p = points[point_index]
    local bad = {}
    local containing = {}

    -- Bowyer--Watson is only topologically safe when the removed cavity is the
    -- connected cavity containing the inserted point.  Exact cocircularities
    -- and edge hits occur constantly in rectangular parameter grids, so collect
    -- every triangle which contains p and force those triangles into the cavity.
    -- This also handles insertion exactly on an interior or hull edge without a
    -- T-junction or a degenerate missing triangle.
    for i, tri in ipairs(triangles) do
        local a,b,c=points[tri[1]],points[tri[2]],points[tri[3]]
        if point_in_triangle(p,a,b,c) then
            containing[#containing+1]=i
        end
        if incircle_contains(a,b,c,p) then
            bad[i]=true
        end
    end
    if #containing==0 then return false end
    for _,i in ipairs(containing) do bad[i]=true end

    -- A mesh can be locally non-Delaunay after an exact cocircular fallback or
    -- after constrained edge recovery.  In that case the in-circle predicate
    -- can report disconnected bad islands.  Removing those islands would punch
    -- holes.  Keep only bad triangles connected to the point-containing seeds.
    local edge_to_bad={}
    local function add_bad_edge(ti,a,b)
        local key=edge_key(a,b)
        local list=edge_to_bad[key]
        if not list then list={}; edge_to_bad[key]=list end
        list[#list+1]=ti
    end
    for ti,tri in ipairs(triangles) do
        if bad[ti] then
            add_bad_edge(ti,tri[1],tri[2])
            add_bad_edge(ti,tri[2],tri[3])
            add_bad_edge(ti,tri[3],tri[1])
        end
    end
    for _,list in pairs(edge_to_bad) do table.sort(list) end

    local connected={}
    local queue={}
    for _,ti in ipairs(containing) do
        if bad[ti] and not connected[ti] then
            connected[ti]=true
            queue[#queue+1]=ti
        end
    end
    local head=1
    while head<=#queue do
        local ti=queue[head]; head=head+1
        local tri=triangles[ti]
        local tri_edges={{tri[1],tri[2]},{tri[2],tri[3]},{tri[3],tri[1]}}
        for _,edge in ipairs(tri_edges) do
            for _,other in ipairs(edge_to_bad[edge_key(edge[1],edge[2])] or {}) do
                if not connected[other] then
                    connected[other]=true
                    queue[#queue+1]=other
                end
            end
        end
    end
    bad=connected

    local edges = {}
    local function add_edge(a, b)
        local key = edge_key(a, b)
        local entry = edges[key]
        if entry then
            entry.count = entry.count + 1
        else
            edges[key] = {a = a, b = b, count = 1}
        end
    end

    local kept = {}
    for i, tri in ipairs(triangles) do
        if bad[i] then
            add_edge(tri[1], tri[2])
            add_edge(tri[2], tri[3])
            add_edge(tri[3], tri[1])
        else
            kept[#kept + 1] = tri
        end
    end

    local boundary_keys = {}
    for key, edge in pairs(edges) do
        if edge.count == 1 then boundary_keys[#boundary_keys + 1] = key end
    end
    table.sort(boundary_keys)
    for _, key in ipairs(boundary_keys) do
        local edge = edges[key]
        local tri = make_triangle(edge.a, edge.b, point_index, points)
        if tri then kept[#kept + 1] = tri end
    end

    for i = 1, #triangles do triangles[i] = nil end
    for i, tri in ipairs(kept) do triangles[i] = tri end
    return true
end

local function build_delaunay(actual_points)
    -- A fixed super-triangle comfortably contains normalized [0,1]^2.
    local points = {
        {x = -128, y = -64, super = true},
        {x =  129, y = -64, super = true},
        {x = 0.5, y = 129, super = true},
    }
    local triangles = {{1, 2, 3}}

    local insertion_failures = 0
    for _, p in ipairs(actual_points) do
        points[#points + 1] = p
        if not delaunay_insert(points, triangles, #points) then
            points[#points] = nil
            insertion_failures = insertion_failures + 1
        end
    end
    if insertion_failures > 0 then
        Recovery.log(
            "adaptive Delaunay recovered",
            ("omitted %d point insertion(s) that could not be embedded")
                :format(insertion_failures)
        )
    end

    return points, triangles, insertion_failures
end

local function real_triangles(triangles)
    local out = {}
    for _, tri in ipairs(triangles) do
        if tri[1] > 3 and tri[2] > 3 and tri[3] > 3 then
            out[#out + 1] = tri
        end
    end
    return out
end

-- Recover a set of non-crossing parameter-space constraint segments by edge
-- flips.  Boundary vertices are inserted with ordinary Delaunay first; the
-- final recovery only changes connectivity, never the triangle budget.  The
-- edge adjacency is maintained incrementally so recovery remains practical for
-- several thousand triangles and hundreds of contour edges.
local function new_edge_map(triangles)
    local edges = {}

    local function add(ti, a, b, opposite)
        local key = edge_key(a, b)
        local entry = edges[key]
        if not entry then
            entry = {a = min(a, b), b = max(a, b), count = 0, triangles = {}}
            edges[key] = entry
        end
        if not entry.triangles[ti] then
            entry.triangles[ti] = opposite
            entry.count = entry.count + 1
        end
    end

    local function add_triangle(ti, tri)
        add(ti, tri[1], tri[2], tri[3])
        add(ti, tri[2], tri[3], tri[1])
        add(ti, tri[3], tri[1], tri[2])
    end

    for ti, tri in ipairs(triangles) do add_triangle(ti, tri) end
    return edges
end

local function remove_edge_triangle(edges, ti, a, b)
    local key = edge_key(a, b)
    local entry = edges[key]
    if not entry or not entry.triangles[ti] then return end
    entry.triangles[ti] = nil
    entry.count = entry.count - 1
    if entry.count == 0 then edges[key] = nil end
end

local function remove_triangle_from_edges(edges, ti, tri)
    remove_edge_triangle(edges, ti, tri[1], tri[2])
    remove_edge_triangle(edges, ti, tri[2], tri[3])
    remove_edge_triangle(edges, ti, tri[3], tri[1])
end

local function add_triangle_to_edges(edges, ti, tri)
    local function add(a, b, opposite)
        local key = edge_key(a, b)
        local entry = edges[key]
        if not entry then
            entry = {a = min(a, b), b = max(a, b), count = 0, triangles = {}}
            edges[key] = entry
        end
        if not entry.triangles[ti] then
            entry.triangles[ti] = opposite
            entry.count = entry.count + 1
        end
    end
    add(tri[1], tri[2], tri[3])
    add(tri[2], tri[3], tri[1])
    add(tri[3], tri[1], tri[2])
end

local function proper_segment_cross(a, b, c, d)
    local o1 = orient2d(a, b, c)
    local o2 = orient2d(a, b, d)
    local o3 = orient2d(c, d, a)
    local o4 = orient2d(c, d, b)
    local e1 = orientation_epsilon(a, b, c)
    local e2 = orientation_epsilon(a, b, d)
    local e3 = orientation_epsilon(c, d, a)
    local e4 = orientation_epsilon(c, d, b)
    return ((o1 > e1 and o2 < -e2) or (o1 < -e1 and o2 > e2))
       and ((o3 > e3 and o4 < -e4) or (o3 < -e3 and o4 > e4))
end

local function intermediate_vertex_on_segment(points, ia, ib)
    local a, b = points[ia], points[ib]
    local dx, dy = b.x - a.x, b.y - a.y
    local denom = dx * dx + dy * dy
    if denom <= NUMERIC_POLICY.minimum_squared_scale then return nil end
    local best, best_t
    for i = 4, #points do
        if i ~= ia and i ~= ib then
            local p = points[i]
            local area = abs(orient2d(a, b, p))
            local eps = orientation_epsilon(
                a, b, p, NUMERIC_POLICY.collinear_relative)
            if area <= eps then
                local t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / denom
                if t > NUMERIC_POLICY.interior_parameter
                    and t < 1 - NUMERIC_POLICY.interior_parameter
                then
                    if not best_t or t < best_t then best, best_t = i, t end
                end
            end
        end
    end
    return best
end

local function recover_constraint(points, triangles, edges, ia, ib, constrained, recursion_depth)
    recursion_depth = recursion_depth or 0
    if recursion_depth > DOMAIN_POLICY.constraint_recursion_limit or ia == ib then return false end
    local wanted = edge_key(ia, ib)
    local max_flips = max(
        DOMAIN_POLICY.constraint_min_flips,
        #triangles * DOMAIN_POLICY.constraint_flip_factor
    )

    for _ = 1, max_flips do
        if edges[wanted] then
            constrained[wanted] = true
            return true
        end

        local pa, pb = points[ia], points[ib]
        local crossing
        local edge_keys={}
        for key in pairs(edges) do edge_keys[#edge_keys+1]=key end
        table.sort(edge_keys)
        for _,key in ipairs(edge_keys) do
            local entry=edges[key]
            if not constrained[key] and entry.count == 2 then
                local u, v = entry.a, entry.b
                if u ~= ia and u ~= ib and v ~= ia and v ~= ib
                    and proper_segment_cross(pa, pb, points[u], points[v])
                then
                    local tis = {}
                    for ti in pairs(entry.triangles) do tis[#tis + 1] = ti end
                    table.sort(tis)
                    local t1i, t2i = tis[1], tis[2]
                    local c = entry.triangles[t1i]
                    local d = entry.triangles[t2i]
                    local side1 = orient2d(points[c], points[d], points[u])
                    local side2 = orient2d(points[c], points[d], points[v])
                    local eps1 = orientation_epsilon(points[c], points[d], points[u])
                    local eps2 = orientation_epsilon(points[c], points[d], points[v])
                    if (side1 > eps1 and side2 < -eps2)
                        or (side1 < -eps1 and side2 > eps2)
                    then
                        crossing = {
                            t1i = t1i, t2i = t2i,
                            u = u, v = v, c = c, d = d,
                        }
                        break
                    end
                end
            end
        end

        if not crossing then
            local mid = intermediate_vertex_on_segment(points, ia, ib)
            if mid then
                return recover_constraint(points, triangles, edges, ia, mid, constrained, recursion_depth + 1)
                   and recover_constraint(points, triangles, edges, mid, ib, constrained, recursion_depth + 1)
            end
            return false
        end

        local old1 = triangles[crossing.t1i]
        local old2 = triangles[crossing.t2i]
        local t1 = make_triangle(crossing.c, crossing.d, crossing.u, points)
        local t2 = make_triangle(crossing.d, crossing.c, crossing.v, points)
        if not (t1 and t2) then return false end

        remove_triangle_from_edges(edges, crossing.t1i, old1)
        remove_triangle_from_edges(edges, crossing.t2i, old2)
        triangles[crossing.t1i] = t1
        triangles[crossing.t2i] = t2
        add_triangle_to_edges(edges, crossing.t1i, t1)
        add_triangle_to_edges(edges, crossing.t2i, t2)
    end
    return false
end

local function recover_constraints(points, triangles, constraints, max_passes)
    local edges = new_edge_map(triangles)
    local constrained = {}
    local pending = {}
    for i, edge in ipairs(constraints) do pending[i] = edge end
    local recovered = 0
    max_passes = max_passes or DOMAIN_POLICY.constraint_recovery_passes

    -- Keep every successfully recovered contour edge fixed while retrying only
    -- the failures.  Rebuilding the whole constrained set on each pass can
    -- otherwise flip an earlier boundary edge back out of the mesh.
    for _ = 1, max_passes do
        if #pending == 0 then break end
        local next_pending = {}
        local progress = false
        for _, edge in ipairs(pending) do
            if recover_constraint(points, triangles, edges,
                edge[1], edge[2], constrained, 0)
            then
                recovered = recovered + 1
                progress = true
            else
                next_pending[#next_pending + 1] = edge
            end
        end
        pending = next_pending
        if not progress then break end
    end

    return recovered, #pending, constrained, pending
end


-- A denser barycentric lattice is used for surfaces than for curves.  In
-- particular this gives narrow ridges and near-singular bands several chances
-- to be discovered before a triangle is declared flat.  The asymmetric probes
-- below break the aliasing which can occur when a feature happens to thread a
-- rational lattice without touching it.
local surface_probes = {}
do
    local order = 6
    for i = 0, order do
        for j = 0, order - i do
            local k = order - i - j
            if not ((i == order and j == 0 and k == 0)
                or (j == order and i == 0 and k == 0)
                or (k == order and i == 0 and j == 0))
            then
                surface_probes[#surface_probes + 1] = {
                    i / order, j / order, k / order,
                }
            end
        end
    end
    local extra = {
        {0.511, 0.293, 0.196},
        {0.196, 0.511, 0.293},
        {0.293, 0.196, 0.511},
        {0.673, 0.211, 0.116},
        {0.116, 0.673, 0.211},
        {0.211, 0.116, 0.673},
    }
    for _, bary in ipairs(extra) do
        surface_probes[#surface_probes + 1] = bary
    end
end

local function triangle_param_diameter(pa, pb, pc)
    local function d(a, b)
        local dx, dy = a.x - b.x, a.y - b.y
        return sqrt(dx * dx + dy * dy)
    end
    return max(d(pa, pb), d(pb, pc), d(pc, pa))
end

local function param_distance(a, b)
    local dx, dy = a.x - b.x, a.y - b.y
    return sqrt(dx * dx + dy * dy)
end

-- ---------------------------------------------------------------------------
-- Adaptive domain-boundary discovery
-- ---------------------------------------------------------------------------
-- Surface interpolation error and domain topology are different problems.  A
-- function can be perfectly flat right up to a non-finite hole, so curvature
-- probes alone cannot be expected to discover that boundary.  The routines
-- below reconstruct finite/non-finite fronts independently in normalized
-- parameter space.  They do not assume an inside, an outside, a preferred
-- direction, or one crossing per scan line; an annulus therefore produces two
-- independent closed contours automatically.

local function locate_validity_transition(a, b, evaluate, tolerance)
    if not a or not b or a.valid == b.valid then return nil, nil end
    local valid_side = a.valid and a or b
    local invalid_side = a.valid and b or a
    -- The boundary mesh may stop at feature_tolerance, but the actual
    -- finite/non-finite transition is a one-dimensional bracket and is cheap
    -- to localize much more accurately.  Keeping this subcell solve tighter
    -- prevents a near-pole finite endpoint from wandering through a large
    -- range of heights merely because the surrounding domain leaf is coarse.
    local transition_tolerance = max(
        tolerance / DOMAIN_POLICY.transition_refinement_divisor,
        NUMERIC_POLICY.transition_floor
    )
    local guard = 0
    while param_distance(valid_side, invalid_side) > transition_tolerance
        and guard < DOMAIN_POLICY.transition_max_iterations
    do
        local mid = evaluate(
            0.5 * (valid_side.x + invalid_side.x),
            0.5 * (valid_side.y + invalid_side.y)
        )
        if mid.valid then valid_side = mid else invalid_side = mid end
        guard = guard + 1
    end
    return valid_side, invalid_side
end

local function discover_domain_boundaries(evaluate, usamples, vsamples, tolerance)
    local cells_u = usamples - 1
    local cells_v = vsamples - 1
    local dx0 = 1 / cells_u
    local dy0 = 1 / cells_v
    local nominal_cell = max(dx0, dy0)

    -- This is only the *discovery* scale.  Once a mixed scout cell is found,
    -- the boundary is refined adaptively down to tolerance.
    local scout_steps = math.ceil(nominal_cell / max(
        DOMAIN_POLICY.scout_feature_multiple * tolerance,
        NUMERIC_POLICY.transition_floor
    ))
    scout_steps = max(
        DOMAIN_POLICY.scout_min_steps,
        min(DOMAIN_POLICY.scout_max_steps, scout_steps)
    )
    local discovery_scale = nominal_cell / scout_steps

    local transition_cache = {}
    local transition_pair_by_valid_key = {}
    local segments = {}
    local segment_seen = {}
    local mixed_scout_cells = 0
    local refined_cells = 0
    local leaf_cells = 0

    local function transition_key(a, b)
        if a.key < b.key then return a.key .. '|' .. b.key end
        return b.key .. '|' .. a.key
    end

    local function edge_transition(a, b)
        if a.valid == b.valid then return nil end
        local key = transition_key(a, b)
        local cached = transition_cache[key]
        if cached then return cached end
        local finite_side, invalid_side = locate_validity_transition(a, b, evaluate, tolerance)
        if not (finite_side and invalid_side) then return nil end
        local item = {valid = finite_side, invalid = invalid_side}
        transition_cache[key] = item
        transition_pair_by_valid_key[finite_side.key] = invalid_side
        return item
    end

    local function add_segment(t1, t2)
        if not (t1 and t2 and t1.valid and t2.valid) then return end
        local a, b = t1.valid, t2.valid
        if a.key == b.key then return end
        local key = transition_key(a, b)
        if segment_seen[key] then return end
        segment_seen[key] = true
        segments[#segments + 1] = {a = a, b = b}
    end

    local function march_square(p00, p10, p11, p01, center)
        local code = (p00.valid and 1 or 0)
            + (p10.valid and 2 or 0)
            + (p11.valid and 4 or 0)
            + (p01.valid and 8 or 0)
        if code == 0 or code == 15 then return end

        local bottom = edge_transition(p00, p10)
        local right  = edge_transition(p10, p11)
        local top    = edge_transition(p11, p01)
        local left   = edge_transition(p01, p00)

        if code == 1 or code == 14 then
            add_segment(bottom, left)
        elseif code == 2 or code == 13 then
            add_segment(bottom, right)
        elseif code == 3 or code == 12 then
            add_segment(right, left)
        elseif code == 4 or code == 11 then
            add_segment(right, top)
        elseif code == 6 or code == 9 then
            add_segment(bottom, top)
        elseif code == 7 or code == 8 then
            add_segment(top, left)
        elseif code == 5 then
            -- Standard marching-squares ambiguity: use a local 2-D decider,
            -- not a fixed diagonal.  This is what lets two nearby contours
            -- remain distinct.
            if center.valid then
                add_segment(bottom, right)
                add_segment(top, left)
            else
                add_segment(bottom, left)
                add_segment(right, top)
            end
        elseif code == 10 then
            if center.valid then
                add_segment(bottom, left)
                add_segment(right, top)
            else
                add_segment(bottom, right)
                add_segment(top, left)
            end
        end
    end

    local function grid3(x0, x1, y0, y1)
        local xm, ym = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        return {
            {evaluate(x0, y0), evaluate(xm, y0), evaluate(x1, y0)},
            {evaluate(x0, ym), evaluate(xm, ym), evaluate(x1, ym)},
            {evaluate(x0, y1), evaluate(xm, y1), evaluate(x1, y1)},
        }
    end

    local function grid_mixed(g)
        local first = g[1][1].valid
        for iy = 1, 3 do
            for ix = 1, 3 do
                if g[iy][ix].valid ~= first then return true end
            end
        end
        return false
    end

    local function emit_leaf(g)
        leaf_cells = leaf_cells + 1
        -- Four quarter-squares prevent an all-same-sign outer square with an
        -- opposite-sign centre from being collapsed to one arbitrary chord.
        march_square(g[1][1], g[1][2], g[2][2], g[2][1],
            evaluate(0.25 * (3*g[1][1].x + g[2][2].x),
                     0.25 * (3*g[1][1].y + g[2][2].y)))
        march_square(g[1][2], g[1][3], g[2][3], g[2][2],
            evaluate(0.25 * (3*g[1][3].x + g[2][2].x),
                     0.25 * (3*g[1][3].y + g[2][2].y)))
        march_square(g[2][2], g[2][3], g[3][3], g[3][2],
            evaluate(0.25 * (3*g[3][3].x + g[2][2].x),
                     0.25 * (3*g[3][3].y + g[2][2].y)))
        march_square(g[2][1], g[2][2], g[3][2], g[3][1],
            evaluate(0.25 * (3*g[3][1].x + g[2][2].x),
                     0.25 * (3*g[3][1].y + g[2][2].y)))
    end

    local function refine_mixed_cell(x0, x1, y0, y1, depth)
        refined_cells = refined_cells + 1
        local g = grid3(x0, x1, y0, y1)
        if not grid_mixed(g) then return end
        local dx, dy = x1 - x0, y1 - y0
        if sqrt(dx*dx + dy*dy) <= 2 * tolerance or depth >= 28 then
            emit_leaf(g)
            return
        end
        local xm, ym = 0.5 * (x0 + x1), 0.5 * (y0 + y1)
        refine_mixed_cell(x0, xm, y0, ym, depth + 1)
        refine_mixed_cell(xm, x1, y0, ym, depth + 1)
        refine_mixed_cell(x0, xm, ym, y1, depth + 1)
        refine_mixed_cell(xm, x1, ym, y1, depth + 1)
    end

    -- A finite scouting lattice is unavoidable for a black-box validity
    -- oracle: a completely hidden component smaller than every sample spacing
    -- cannot be guaranteed detectable.  Crucially, this uniform pass only
    -- *discovers* mixed micro-cells; all expensive localization is adaptive and
    -- local to the boundary.
    for i = 0, cells_u - 1 do
        local cell_x0 = i / cells_u
        local cell_x1 = (i + 1) / cells_u
        for j = 0, cells_v - 1 do
            local cell_y0 = j / cells_v
            local cell_y1 = (j + 1) / cells_v
            local samples = {}
            for ix = 0, scout_steps do
                samples[ix] = {}
                local x = cell_x0 + (cell_x1 - cell_x0) * ix / scout_steps
                for iy = 0, scout_steps do
                    local y = cell_y0 + (cell_y1 - cell_y0) * iy / scout_steps
                    samples[ix][iy] = evaluate(x, y)
                end
            end
            for ix = 0, scout_steps - 1 do
                for iy = 0, scout_steps - 1 do
                    local x0 = cell_x0 + (cell_x1 - cell_x0) * ix / scout_steps
                    local x1 = cell_x0 + (cell_x1 - cell_x0) * (ix + 1) / scout_steps
                    local y0 = cell_y0 + (cell_y1 - cell_y0) * iy / scout_steps
                    local y1 = cell_y0 + (cell_y1 - cell_y0) * (iy + 1) / scout_steps
                    local p00 = samples[ix][iy]
                    local p10 = samples[ix + 1][iy]
                    local p11 = samples[ix + 1][iy + 1]
                    local p01 = samples[ix][iy + 1]
                    local center = evaluate(0.5*(x0+x1), 0.5*(y0+y1))
                    local first = p00.valid
                    local mixed = p10.valid ~= first or p11.valid ~= first
                        or p01.valid ~= first or center.valid ~= first
                    if not mixed then
                        -- Symmetric-ish irrational offsets catch oblique/narrow
                        -- fronts without privileging x, y, or any radial ray.
                        local pA = evaluate(x0 + 0.211324865405187*(x1-x0),
                                            y0 + 0.677419354838710*(y1-y0))
                        local pB = evaluate(x0 + 0.788675134594813*(x1-x0),
                                            y0 + 0.322580645161290*(y1-y0))
                        mixed = pA.valid ~= first or pB.valid ~= first
                    end
                    if mixed then
                        mixed_scout_cells = mixed_scout_cells + 1
                        refine_mixed_cell(x0, x1, y0, y1, 0)
                    end
                end
            end
        end
    end

    -- Count connected contour components for diagnostics only.
    local parent = {}
    local function find(k)
        local p = parent[k]
        if not p then parent[k] = k; return k end
        if p ~= k then parent[k] = find(p) end
        return parent[k]
    end
    local function union(a, b)
        a, b = find(a), find(b)
        if a ~= b then parent[b] = a end
    end
    for _, segment in ipairs(segments) do union(segment.a.key, segment.b.key) end
    local roots = {}
    for key in pairs(parent) do roots[find(key)] = true end
    local components = 0
    for _ in pairs(roots) do components = components + 1 end

    return {
        segments = segments,
        components = components,
        discovery_scale = discovery_scale,
        scout_steps = scout_steps,
        mixed_scout_cells = mixed_scout_cells,
        refined_cells = refined_cells,
        leaf_cells = leaf_cells,
        transition_pair_by_valid_key = transition_pair_by_valid_key,
    }
end

local function simplify_domain_boundary(domain, tolerance)
    local segments = domain.segments or {}
    if #segments == 0 then
        domain.points = {}
        return domain
    end

    local point_by_key = {}
    local adjacency = {}

    for _, segment in ipairs(segments) do
        local endpoints = {segment.a, segment.b}
        for _, point in ipairs(endpoints) do
            point_by_key[point.key] = point
            adjacency[point.key] = adjacency[point.key] or {}
        end
        adjacency[segment.a.key][#adjacency[segment.a.key] + 1] = segment.b.key
        adjacency[segment.b.key][#adjacency[segment.b.key] + 1] = segment.a.key
    end

    local adjacency_keys = {}
    for key, neighbors in pairs(adjacency) do
        table.sort(neighbors)
        adjacency_keys[#adjacency_keys + 1] = key
    end
    table.sort(adjacency_keys)

    local function segment_key(a, b)
        if a < b then return a .. "|" .. b end
        return b .. "|" .. a
    end

    local used = {}
    local paths = {}

    local function walk(start_key, next_key)
        local keys = {start_key, next_key}
        local previous = start_key
        local current = next_key
        used[segment_key(previous, current)] = true

        local guard = 0
        while guard < #segments + 2 do
            guard = guard + 1
            local candidate
            for _, neighbor in ipairs(adjacency[current] or {}) do
                if not used[segment_key(current, neighbor)] then
                    candidate = neighbor
                    break
                end
            end
            if not candidate then break end

            previous, current = current, candidate
            used[segment_key(previous, current)] = true
            keys[#keys + 1] = current
            if current == start_key then break end
        end
        return keys
    end

    -- Open and branched paths first, then any remaining closed loops.
    for _, key in ipairs(adjacency_keys) do
        local neighbors = adjacency[key]
        if #neighbors ~= 2 then
            for _, neighbor in ipairs(neighbors) do
                if not used[segment_key(key, neighbor)] then
                    paths[#paths + 1] = walk(key, neighbor)
                end
            end
        end
    end
    for _, segment in ipairs(segments) do
        if not used[segment_key(segment.a.key, segment.b.key)] then
            paths[#paths + 1] = walk(segment.a.key, segment.b.key)
        end
    end

    local function point_segment_distance_2d(point, a, b)
        local dx = b.x - a.x
        local dy = b.y - a.y
        local denominator = dx * dx + dy * dy
        if denominator <= NUMERIC_POLICY.minimum_squared_scale then
            return param_distance(point, a)
        end

        local t = ((point.x - a.x) * dx + (point.y - a.y) * dy) / denominator
        t = max(0, min(1, t))
        local ex = point.x - (a.x + t * dx)
        local ey = point.y - (a.y + t * dy)
        return sqrt(ex * ex + ey * ey)
    end

    local function rdp(points, epsilon)
        if #points <= 2 then return points end

        local keep = {[1] = true, [#points] = true}
        local stack = {{1, #points}}

        while #stack > 0 do
            local range = stack[#stack]
            stack[#stack] = nil
            local first = range[1]
            local last = range[2]
            local worst_distance = -1
            local worst_index

            for i = first + 1, last - 1 do
                local distance = point_segment_distance_2d(
                    points[i], points[first], points[last])
                if distance > worst_distance then
                    worst_distance = distance
                    worst_index = i
                end
            end

            if worst_index and worst_distance > epsilon then
                keep[worst_index] = true
                stack[#stack + 1] = {first, worst_index}
                stack[#stack + 1] = {worst_index, last}
            end
        end

        local reduced = {}
        for i = 1, #points do
            if keep[i] then reduced[#reduced + 1] = points[i] end
        end
        return reduced
    end

    local function closed_simplify(points, epsilon)
        local count = #points
        if count <= 4 then return points end

        -- Split the loop at an approximately diametric pair, simplify the two
        -- open arcs independently, then join them back into a closed polygon.
        local first_index = 1
        local farthest = -1
        for i = 2, count do
            local distance = param_distance(points[1], points[i])
            if distance > farthest then
                farthest = distance
                first_index = i
            end
        end

        local second_index = first_index == 1 and 2 or 1
        farthest = -1
        for i = 1, count do
            if i ~= first_index then
                local distance = param_distance(points[first_index], points[i])
                if distance > farthest then
                    farthest = distance
                    second_index = i
                end
            end
        end
        if first_index > second_index then
            first_index, second_index = second_index, first_index
        end

        local first_arc = {}
        local second_arc = {}
        for i = first_index, second_index do
            first_arc[#first_arc + 1] = points[i]
        end
        for i = second_index, count do
            second_arc[#second_arc + 1] = points[i]
        end
        for i = 1, first_index do
            second_arc[#second_arc + 1] = points[i]
        end

        first_arc = rdp(first_arc, epsilon)
        second_arc = rdp(second_arc, epsilon)

        local reduced = {}
        for _, point in ipairs(first_arc) do
            reduced[#reduced + 1] = point
        end
        for i = 2, #second_arc - 1 do
            reduced[#reduced + 1] = second_arc[i]
        end
        return reduced
    end

    local output_segments = {}
    local output_points = {}
    local seen_segments = {}
    local seen_points = {}

    local function add_point(point)
        if seen_points[point.key] then return end
        seen_points[point.key] = true
        output_points[#output_points + 1] = point
    end

    local function add_segment(a, b)
        if not a or not b or a.key == b.key then return end
        local key = segment_key(a.key, b.key)
        if seen_segments[key] then return end
        seen_segments[key] = true
        output_segments[#output_segments + 1] = {a = a, b = b}
        add_point(a)
        add_point(b)
    end

    for _, keys in ipairs(paths) do
        local closed = #keys >= 2 and keys[1] == keys[#keys]
        if closed then keys[#keys] = nil end

        local points = {}
        local branched = false
        for index, key in ipairs(keys) do
            points[#points + 1] = point_by_key[key]
            local degree = #(adjacency[key] or {})
            if closed then
                if degree ~= 2 then branched = true end
            else
                local endpoint = index == 1 or index == #keys
                if (endpoint and degree ~= 1) or ((not endpoint) and degree ~= 2) then
                    branched = true
                end
            end
        end

        local reduced = points
        if not branched then
            if closed then
                reduced = closed_simplify(points, tolerance)
            else
                reduced = rdp(points, tolerance)
            end
        end

        for i = 1, #reduced - 1 do
            add_segment(reduced[i], reduced[i + 1])
        end
        if closed and #reduced >= 3 then
            add_segment(reduced[#reduced], reduced[1])
        end
    end

    domain.raw_segments = #segments
    domain.segments = output_segments
    domain.points = output_points
    return domain
end


local function closest_valid_invalid_boundary(valid_samples, invalid_samples, evaluate, tolerance)
    local valid_side, invalid_side
    local best_distance = huge
    for _, a in ipairs(valid_samples) do
        for _, b in ipairs(invalid_samples) do
            local distance = param_distance(a, b)
            if distance < best_distance then
                best_distance = distance
                valid_side, invalid_side = a, b
            end
        end
    end
    if not (valid_side and invalid_side) then return nil end

    -- Locate the validity transition numerically before spending a mesh vertex
    -- on it.  The old implementation inserted one midpoint per refinement;
    -- with a low max_depth that left asymptotes and clipped contours as coarse
    -- saw teeth.  Bisection makes feature_tolerance mean what it says: the
    -- desired parameter-space localization of the non-finite boundary.
    local guard = 0
    while param_distance(valid_side, invalid_side) > tolerance and guard < 64 do
        local mid = evaluate(
            0.5 * (valid_side.x + invalid_side.x),
            0.5 * (valid_side.y + invalid_side.y)
        )
        if mid.valid then
            valid_side = mid
        else
            invalid_side = mid
        end
        guard = guard + 1
    end

    return invalid_side
end

local function surface_triangle_score(tri, points, evaluate, feature_tolerance)
    local pa, pb, pc = points[tri[1]], points[tri[2]], points[tri[3]]
    local va, vb, vc = pa.metric, pb.metric, pc.metric
    local vertices_valid = pa.valid and pb.valid and pc.valid
    local best_error = -1
    local best_probe
    local unsafe = not vertices_valid
    local valid_samples = {}
    local invalid_samples = {}

    local function remember(sample)
        if sample.valid then
            valid_samples[#valid_samples + 1] = sample
        else
            invalid_samples[#invalid_samples + 1] = sample
        end
    end
    remember(pa)
    remember(pb)
    remember(pc)

    for _, bary in ipairs(surface_probes) do
        local wa, wb, wc = bary[1], bary[2], bary[3]
        local x = wa * pa.x + wb * pb.x + wc * pc.x
        local y = wa * pa.y + wb * pb.y + wc * pc.y
        local probe = evaluate(x, y)
        remember(probe)

        if not probe.valid then
            unsafe = true
        elseif vertices_valid then
            local planar = bary_point(va, vb, vc, wa, wb, wc)
            local err = point_distance(probe.metric, planar)
            if err > best_error then
                best_error = err
                best_probe = probe
            end
        end
    end

    if unsafe then
        -- Once both sides of a finite/non-finite boundary have been observed,
        -- refine the *bracket* rather than repeatedly dropping arbitrary
        -- samples into the invalid region.  This converges on holes, clipping
        -- contours and asymptotic bands much more cleanly.
        best_error = huge
        best_probe = closest_valid_invalid_boundary(
            valid_samples, invalid_samples, evaluate, feature_tolerance)
            or best_probe
            or invalid_samples[1]
            or valid_samples[1]
    elseif best_error < 0 then
        best_error = huge
    end

    return {
        error = best_error,
        unsafe = unsafe,
        probe = best_probe,
        diameter = triangle_param_diameter(pa, pb, pc),
    }
end

function Adaptive.surface(
    f,
    ustart, ustop, usamples,
    vstart, vstop, vsamples,
    user_spec,
    metric
)
    assert(Vector ~= nil, "Adaptive classes have not been initialized")
    assert(usamples >= 2 and vsamples >= 2,
        "adaptive surface requires at least two initial samples per axis")

    local initial_triangles = 2 * (usamples - 1) * (vsamples - 1)
    local spec = normalize_spec(user_spec, "surface", initial_triangles)
    local initial_dx = 1 / (usamples - 1)
    local initial_dy = 1 / (vsamples - 1)
    local initial_diameter = sqrt(initial_dx * initial_dx + initial_dy * initial_dy)
    local min_diameter = initial_diameter / (2 ^ spec.max_depth)
    local eval_cache = {}
    local point_index_by_key = {}

    local function evaluate(x, y)
        x, y = clamp01(x), clamp01(y)
        local key = point_key(x, y)
        local cached = eval_cache[key]
        if cached then return cached end

        local u = axis_value(ustart, ustop, x)
        local v = axis_value(vstart, vstop, y)
        local ok, value = pcall(f, u, v)
        if not ok then
            Recovery.log_once("adaptive-surface-sample", "adaptive surface sample omitted", value)
            value = nil
        end
        local measured
        if finite_vector(value) then
            if metric then
                local metric_ok, metric_value = pcall(metric, value)
                if metric_ok then
                    measured = metric_value
                else
                    Recovery.log_once(
                        "adaptive-surface-metric",
                        "adaptive surface metric sample omitted",
                        metric_value
                    )
                end
            else
                measured = value
            end
        end
        local p = {
            x = x,
            y = y,
            u = u,
            v = v,
            value = value,
            metric = measured,
            valid = finite_vector(value) and finite_vector(measured),
            key = key,
        }
        eval_cache[key] = p
        return p
    end

    local initial_points = {}
    -- Insert the grid in a checkerboard-like order rather than row-major order;
    -- this reduces degeneracy bias for cocircular rectangular grids while all
    -- coordinates remain exact.
    for parity = 0, 1 do
        for i = 0, usamples - 1 do
            for j = 0, vsamples - 1 do
                if (i + j) % 2 == parity then
                    local x = i / (usamples - 1)
                    local y = j / (vsamples - 1)
                    local p = evaluate(x, y)
                    initial_points[#initial_points + 1] = p
                end
            end
        end
    end

    local points, triangles, initial_insertion_failures = build_delaunay(initial_points)
    for i = 4, #points do
        point_index_by_key[points[i].key] = i
    end

    local current = real_triangles(triangles)
    if #current > spec.budget then
        Recovery.log(
            "adaptive surface budget recovered",
            ("initial triangulation required %d triangles; raising the local budget from %d")
                :format(#current, spec.budget)
        )
        spec.budget = #current
    end

    -- Most Delaunay triangles survive each point insertion unchanged.  Their
    -- error estimates depend only on their three vertices and the sampled
    -- surface, so cache them by vertex set.  Without this cache every
    -- refinement rescored the entire current mesh, making a ~1000 triangle
    -- budget needlessly expensive.
    local score_cache = {}
    local function triangle_key(tri)
        local a, b, c = tri[1], tri[2], tri[3]
        if a > b then a, b = b, a end
        if b > c then b, c = c, b end
        if a > b then a, b = b, a end
        return a .. ":" .. b .. ":" .. c
    end
    local function score_triangle(tri)
        local key = triangle_key(tri)
        local score = score_cache[key]
        if not score then
            score = surface_triangle_score(tri, points, evaluate, spec.feature_tolerance)
            score_cache[key] = score
        end
        return score
    end

    local exhausted = false
    local blocked_triangles = {}
    local blocked_count = 0

    local function refinable(score)
        if score.unsafe then
            return score.diameter > spec.feature_tolerance
        end
        if score.diameter > min_diameter then
            return true
        end
        -- max_depth is the ordinary smooth-surface limit, not a reason to
        -- abandon a sharp feature.  If a triangle is still badly above the
        -- requested error at that scale, allow it to keep shrinking toward
        -- feature_tolerance.  The surface-local triangle budget remains the
        -- hard cap.
        return score.error > 16 * spec.tolerance
            and score.diameter > spec.feature_tolerance
    end

    local function refinement_priority(score)
        -- Compare unlike reasons for refinement in dimensionless units.  An
        -- unsafe feature no longer receives infinite priority forever, which
        -- previously starved perfectly finite ridges whenever a singularity
        -- existed elsewhere on the same surface.
        if score.unsafe then
            return score.diameter / spec.feature_tolerance
        end
        return score.error / spec.tolerance
    end

    while #current + 2 <= spec.budget do
        local worst
        local worst_tri
        local worst_key
        local worst_priority = 1

        for _, tri in ipairs(current) do
            local key = triangle_key(tri)
            if not blocked_triangles[key] then
                local score = score_triangle(tri)
                local needs_refinement = score.unsafe or score.error > spec.tolerance
                if needs_refinement and refinable(score) and score.probe then
                    local priority = refinement_priority(score)
                    if priority > worst_priority then
                        worst_priority = priority
                        worst = score
                        worst_tri = tri
                        worst_key = key
                    end
                end
            end
        end

        if not worst then break end

        local candidate = worst.probe
        if point_index_by_key[candidate.key] then
            -- Try the centroid if the best probe was already inserted.  If
            -- even that point is unavailable, only this triangle is blocked;
            -- refinement continues elsewhere instead of aborting the entire
            -- surface.
            local a, b, c = points[worst_tri[1]], points[worst_tri[2]], points[worst_tri[3]]
            candidate = evaluate(
                (a.x + b.x + c.x) / 3,
                (a.y + b.y + c.y) / 3
            )
        end

        if point_index_by_key[candidate.key] then
            blocked_triangles[worst_key] = true
            blocked_count = blocked_count + 1
        else
            points[#points + 1] = candidate
            local new_index = #points
            point_index_by_key[candidate.key] = new_index
            if not delaunay_insert(points, triangles, new_index) then
                points[new_index] = nil
                point_index_by_key[candidate.key] = nil
                blocked_triangles[worst_key] = true
                blocked_count = blocked_count + 1
            else
                current = real_triangles(triangles)
            end
        end
    end

    -- Curvature sampling is completely finished before domain removal.  The
    -- domain stage below overlays a constrained contour onto that finished mesh.
    -- It never asks the surface for a new refinement sample: boundary discovery
    -- only evaluates validity, and the overlay only changes connectivity.
    local curvature_triangles = #current
    local curvature_point_count = #points
    local domain = simplify_domain_boundary(
        discover_domain_boundaries(
            evaluate, usamples, vsamples, spec.feature_tolerance),
        spec.feature_tolerance / DOMAIN_POLICY.simplify_tolerance_divisor
    )

    -- Preserve the finished curvature triangles as the starting connectivity.
    -- Boundary and collar vertices are inserted through local Delaunay cavities.
    -- This is deliberately not a second global triangulation: connectivity can
    -- change only in the local cavity of an inserted post-process point.
    local sewn = {}
    for i, tri in ipairs(current) do
        sewn[i] = {tri[1], tri[2], tri[3]}
    end

    local boundary_vertices_inserted = 0
    local boundary_vertex_failures = 0
    local collar_vertices_inserted = 0
    local collar_vertex_failures = 0

    local function insert_postprocess_vertex(point, required_boundary)
        local old = point_index_by_key[point.key]
        if old then return old end

        points[#points + 1] = point
        local point_index = #points
        point_index_by_key[point.key] = point_index

        if not delaunay_insert(points, sewn, point_index) then
            points[point_index] = nil
            point_index_by_key[point.key] = nil
            if required_boundary then
                boundary_vertex_failures = boundary_vertex_failures + 1
            else
                collar_vertex_failures = collar_vertex_failures + 1
            end
            return nil
        end

        if required_boundary then
            boundary_vertices_inserted = boundary_vertices_inserted + 1
        else
            collar_vertices_inserted = collar_vertices_inserted + 1
        end
        return point_index
    end

    -- Insert each reconstructed contour vertex exactly once.  Sorting makes the
    -- result deterministic even if Lua table traversal order changes.
    local boundary_points = {}
    for _, point in ipairs(domain.points or {}) do
        boundary_points[#boundary_points + 1] = point
    end
    table.sort(boundary_points, function(a, b) return a.key < b.key end)
    for _, point in ipairs(boundary_points) do
        insert_postprocess_vertex(point, true)
    end

    -- Build a purely numerical finite-side transition collar around every
    -- reconstructed domain boundary.  This is independent of the cause of the
    -- restriction: an ordinary hole, clipped pole, transcendental failure, or
    -- user-defined invalid set all receive the same treatment.
    --
    -- Boundary vertices alone are too dense relative to the already-finished
    -- curvature mesh and create long Delaunay fans.  At each boundary vertex,
    -- walk away from its paired invalid sample into the finite region with
    -- geometrically increasing offsets.  Stop once the collar reaches the local
    -- spacing of the original curvature vertices.  Thus the post-process
    -- bridges scales smoothly without resampling or changing curvature logic.
    local collar_start = DOMAIN_POLICY.collar_start_feature_multiple
        * spec.feature_tolerance
    local collar_target_fraction = DOMAIN_POLICY.collar_target_spacing_fraction
    local collar_initial_cell_cap = DOMAIN_POLICY.collar_initial_cell_fraction
        * min(initial_dx, initial_dy)
    local collar_max_layers = DOMAIN_POLICY.collar_max_layers

    local boundary_point_by_key = {}
    for _, segment in ipairs(domain.segments or {}) do
        boundary_point_by_key[segment.a.key] = segment.a
        boundary_point_by_key[segment.b.key] = segment.b
    end

    local collar_points = {}
    local collar_seen = {}
    local boundary_keys = {}
    for key in pairs(boundary_point_by_key) do
        boundary_keys[#boundary_keys + 1] = key
    end
    table.sort(boundary_keys)

    for _, key in ipairs(boundary_keys) do
        local point = boundary_point_by_key[key]
        local invalid = domain.transition_pair_by_valid_key
            and domain.transition_pair_by_valid_key[key]

        if invalid then
            local nx = point.x - invalid.x
            local ny = point.y - invalid.y
            local normal_length = sqrt(nx * nx + ny * ny)

            if normal_length > sqrt(NUMERIC_POLICY.minimum_squared_scale) then
                nx = nx / normal_length
                ny = ny / normal_length

                local nearest = huge
                for i = 4, curvature_point_count do
                    local candidate = points[i]
                    if candidate and candidate.valid then
                        local distance = param_distance(point, candidate)
                        if distance < nearest then nearest = distance end
                    end
                end

                local target = min(
                    collar_target_fraction * nearest,
                    collar_initial_cell_cap
                )
                local distance = collar_start
                local layer = 0

                while distance < target and layer < collar_max_layers do
                    local candidate = evaluate(
                        point.x + nx * distance,
                        point.y + ny * distance
                    )
                    if candidate.valid
                        and not point_index_by_key[candidate.key]
                        and not collar_seen[candidate.key]
                    then
                        collar_seen[candidate.key] = true
                        collar_points[#collar_points + 1] = candidate
                    end
                    distance = distance * 2
                    layer = layer + 1
                end
            end
        end
    end

    table.sort(collar_points, function(a, b) return a.key < b.key end)
    for _, point in ipairs(collar_points) do
        insert_postprocess_vertex(point, false)
    end

    -- Every simplified contour segment must either become a recoverable mesh
    -- constraint or enter the fail-closed path below.  A failed contour-vertex
    -- insertion must never silently make a domain segment disappear.
    local constraints = {}
    local seen_constraints = {}
    local missing_constraint_segments = {}

    for _, segment in ipairs(domain.segments or {}) do
        local ia = point_index_by_key[segment.a.key]
        local ib = point_index_by_key[segment.b.key]

        if ia and ib and ia ~= ib then
            local key = edge_key(ia, ib)
            if not seen_constraints[key] then
                seen_constraints[key] = true
                constraints[#constraints + 1] = {ia, ib}
            end
        else
            missing_constraint_segments[#missing_constraint_segments + 1] = {
                a = segment.a,
                b = segment.b,
            }
        end
    end

    table.sort(constraints, function(a, b)
        return edge_key(a[1], a[2]) < edge_key(b[1], b[2])
    end)

    local constraints_recovered = 0
    local constraint_recovery_failures = 0
    local constrained_edges = {}
    local failed_constraints = {}

    if #constraints > 0 then
        constraints_recovered,
        constraint_recovery_failures,
        constrained_edges,
        failed_constraints = recover_constraints(
            points,
            sewn,
            constraints,
            DOMAIN_POLICY.constraint_recovery_passes
        )
    end

    local failed_constraint_segments = {}
    for _, segment in ipairs(missing_constraint_segments) do
        failed_constraint_segments[#failed_constraint_segments + 1] = segment
    end
    for _, edge in ipairs(failed_constraints) do
        failed_constraint_segments[#failed_constraint_segments + 1] = {
            a = points[edge[1]],
            b = points[edge[2]],
        }
    end

    local constraints_failed = constraint_recovery_failures
        + #missing_constraint_segments

    -- Constraint recovery is normally exact.  If vertex insertion or an edge
    -- flip fails, fail closed: remove triangles touched by the unrecovered
    -- contour segment instead of permitting a bridge across the invalid set.
    local forced_discards = {}
    if #failed_constraint_segments > 0 then
        for triangle_index, triangle in ipairs(sewn) do
            local a = points[triangle[1]]
            local b = points[triangle[2]]
            local c = points[triangle[3]]

            for _, segment in ipairs(failed_constraint_segments) do
                local p = segment.a
                local q = segment.b
                if point_in_triangle(p, a, b, c)
                    or point_in_triangle(q, a, b, c)
                    or proper_segment_cross(p, q, a, b)
                    or proper_segment_cross(p, q, b, c)
                    or proper_segment_cross(p, q, c, a)
                then
                    forced_discards[triangle_index] = true
                    break
                end
            end
        end
    end

    -- The recovered contour is the domain boundary.  Classify whole connected
    -- triangle components separated by those constrained edges, rather than
    -- reclassifying every boundary triangle independently.  This prevents a
    -- second jagged boundary from appearing inside the smooth numerical contour.
    local mesh_edges = new_edge_map(sewn)
    local neighbors = {}
    for i = 1, #sewn do
        neighbors[i] = {}
    end

    local mesh_edge_keys = {}
    for key in pairs(mesh_edges) do
        mesh_edge_keys[#mesh_edge_keys + 1] = key
    end
    table.sort(mesh_edge_keys)

    for _, key in ipairs(mesh_edge_keys) do
        local entry = mesh_edges[key]
        if entry.count == 2 and not constrained_edges[key] then
            local triangle_indices = {}
            for triangle_index in pairs(entry.triangles) do
                triangle_indices[#triangle_indices + 1] = triangle_index
            end
            table.sort(triangle_indices)

            local first = triangle_indices[1]
            local second = triangle_indices[2]
            if not forced_discards[first] and not forced_discards[second] then
                neighbors[first][#neighbors[first] + 1] = second
                neighbors[second][#neighbors[second] + 1] = first
            end
        end
    end
    for _, list in ipairs(neighbors) do
        table.sort(list)
    end

    local component_of = {}
    local components = {}
    for seed = 1, #sewn do
        if not forced_discards[seed] and not component_of[seed] then
            local component_index = #components + 1
            local members = {}
            local queue = {seed}
            local head = 1
            component_of[seed] = component_index

            while head <= #queue do
                local triangle_index = queue[head]
                head = head + 1
                members[#members + 1] = triangle_index

                for _, other in ipairs(neighbors[triangle_index]) do
                    if not component_of[other] then
                        component_of[other] = component_index
                        queue[#queue + 1] = other
                    end
                end
            end
            components[component_index] = members
        end
    end

    local component_valid = {}
    for component_index, members in ipairs(components) do
        -- Vote with the largest cells first.  A component is topologically on
        -- one side of the recovered contour, so a few interior probes are much
        -- more stable than making a separate keep/discard decision per facet.
        local ranked = {}
        for _, triangle_index in ipairs(members) do
            local triangle = sewn[triangle_index]
            local a = points[triangle[1]]
            local b = points[triangle[2]]
            local c = points[triangle[3]]
            ranked[#ranked + 1] = {
                triangle_index = triangle_index,
                area = abs(orient2d(a, b, c)),
            }
        end
        table.sort(ranked, function(a, b)
            if a.area == b.area then
                return a.triangle_index < b.triangle_index
            end
            return a.area > b.area
        end)

        local valid_votes = 0
        local invalid_votes = 0
        local probe_count = min(7, #ranked)
        for i = 1, probe_count do
            local triangle = sewn[ranked[i].triangle_index]
            local a = points[triangle[1]]
            local b = points[triangle[2]]
            local c = points[triangle[3]]
            local probe = evaluate(
                (a.x + b.x + c.x) / 3,
                (a.y + b.y + c.y) / 3
            )
            if probe.valid then
                valid_votes = valid_votes + 1
            else
                invalid_votes = invalid_votes + 1
            end
        end
        component_valid[component_index] = valid_votes > invalid_votes
    end

    local output = {}
    local domain_discards = 0
    local mixed_component_discards = 0
    for triangle_index, triangle in ipairs(sewn) do
        local component_index = component_of[triangle_index]
        local a = points[triangle[1]]
        local b = points[triangle[2]]
        local c = points[triangle[3]]

        if component_index
            and component_valid[component_index]
            and a.valid and b.valid and c.valid
        then
            output[#output + 1] = {
                points = {a.value, b.value, c.value},
                uv = {
                    Vector:_new{a.u, a.v, 1},
                    Vector:_new{b.u, b.v, 1},
                    Vector:_new{c.u, c.v, 1},
                },
            }
        else
            domain_discards = domain_discards + 1
            if component_index
                and component_valid[component_index]
                and not (a.valid and b.valid and c.valid)
            then
                mixed_component_discards = mixed_component_discards + 1
            end
        end
    end

    -- Diagnostics about geometric refinement still refer to the completed
    -- curvature mesh, not to the post-domain connectivity overlay.
    local worst_error = 0
    local unmet = 0
    local unresolved = 0
    local still_refinable = false
    for _, tri in ipairs(current) do
        local score = score_triangle(tri)
        if score.unsafe then
            unresolved = unresolved + 1
        else
            if score.error > worst_error then worst_error = score.error end
            if score.error > spec.tolerance then unmet = unmet + 1 end
        end
        if (score.unsafe or score.error > spec.tolerance) and refinable(score) then
            still_refinable = true
        end
    end
    if (#current + 2 > spec.budget) and still_refinable then exhausted = true end
    if #output > spec.budget then exhausted = true end

    return output, {
        triangles = curvature_triangles,
        curvature_triangles = curvature_triangles,
        sewn_triangles = #sewn,
        rendered_triangles = #output,
        worst_error = worst_error,
        unresolved = unresolved,
        unmet = unmet,
        exhausted = exhausted,
        tolerance = spec.tolerance,
        min_diameter = min_diameter,
        blocked = blocked_count,
        initial_delaunay_insertion_failures = initial_insertion_failures or 0,
        boundary_components = domain.components,
        boundary_segments = #(domain.segments or {}),
        boundary_segments_raw = domain.raw_segments or #(domain.segments or {}),
        boundary_discovery_scale = domain.discovery_scale,
        boundary_scout_steps = domain.scout_steps,
        boundary_vertices_inserted = boundary_vertices_inserted,
        boundary_vertex_failures = boundary_vertex_failures,
        boundary_collar_points = #collar_points,
        boundary_collar_vertices_inserted = collar_vertices_inserted,
        boundary_collar_vertex_failures = collar_vertex_failures,
        constraints_attempted = #(domain.segments or {}),
        constraints_recovered = constraints_recovered,
        constraint_recovery_failures = constraint_recovery_failures,
        constraint_segments_missing = #missing_constraint_segments,
        constraints_failed = constraints_failed,
        domain_mesh_components = #components,
        forced_domain_discards = table_key_count(forced_discards),
        mixed_component_discards = mixed_component_discards,
        domain_discards = domain_discards,
        boundary_overhead = max(0, #output - curvature_triangles),
        domain_topology_safe = constraints_failed == 0
            and boundary_vertex_failures == 0,
    }

end

return Adaptive
