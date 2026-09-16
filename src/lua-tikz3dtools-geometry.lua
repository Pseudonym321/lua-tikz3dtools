--- Geometry class
--- @class Geometry
local Geometry = {}
Geometry.__index = Geometry

local AABBTree = require "lua-tikz3dtools-aabbtree"
local Recovery = require "lua-tikz3dtools-recovery"

local function log_line(message)
    local prefix = ("Time:%s "):format(os.date("%X"))
    local body = tostring(message):gsub("%s+", " "):gsub("^%s+", ""):gsub("%s+$", "")
    local room = 76 - #prefix
    if #body > room then body = body:sub(1, math.max(1, room - 3)) .. "..." end
    local line = prefix .. body
    if texio and texio.write_nl then
        texio.write_nl("term and log", line)
    else
        io.stdout:write(line, "\n")
        io.stdout:flush()
    end
end

local Vector = nil
local Matrix = nil

-- Geometric predicates operate in caller coordinates, not normalized parameter
-- space.  Tolerances therefore scale with the geometry being tested instead of
-- assuming that one drawing unit is a privileged size.
local GEOMETRY_REL_EPS = 1e-12
local PARTITION_REL_EPS = 1e-10
local MERGE_REL_EPS = 1e-7
local MIN_GEOMETRY_SCALE = 1e-150

local function coordinate_scale(...)
    local scale = MIN_GEOMETRY_SCALE
    for i = 1, select("#", ...) do
        local point = select(i, ...)
        if point then
            for j = 1, #point - 1 do
                scale = math.max(scale, math.abs(point[j]))
            end
        end
    end
    return scale
end

local function point_epsilon(relative, ...)
    return (relative or GEOMETRY_REL_EPS) * coordinate_scale(...)
end

local function simplex_length_scale(simplex)
    local scale = MIN_GEOMETRY_SCALE
    for i = 1, #simplex do
        local a = Vector:_new(simplex[i])
        for j = i + 1, #simplex do
            local b = Vector:_new(simplex[j])
            scale = math.max(scale, a:_hdistance(b))
        end
    end
    return scale
end

local function simplex_length_epsilon(simplex, relative)
    local scale = MIN_GEOMETRY_SCALE
    for i = 1, #simplex do
        local point = simplex[i]
        for j = 1, #point - 1 do
            scale = math.max(scale, math.abs(point[j]))
        end
    end
    return (relative or GEOMETRY_REL_EPS) * scale
end

local function triangle_area_epsilon(triangle, relative)
    local scale = simplex_length_scale(triangle)
    return (relative or GEOMETRY_REL_EPS) * scale * scale
end

--- Vector and Matrix injection
--- @param vclass Vector the class is its own metatable
function Geometry:_set_classes(vclass, matclass)
    Vector = vclass
    Matrix = matclass
end


--- Check if two homogeneous vectors intersect (are the same point)
--- @param self Vector
--- @param other Vector
--- @return boolean
function Geometry.hpoint_point_intersecting(self, other)
    return self:_hdistance(other) <= point_epsilon(GEOMETRY_REL_EPS, self, other)
end

--- Check if a homogeneous point is in the triangular prism defined by triangle T
--- @param self Vector
--- @param T Matrix
--- @return boolean
function Geometry.hpoint_in_triangular_prism(self, T)
    local bary = Geometry.hpoint_triangle_barycentric(self, T)
    if not bary then return false end
    local eps = 3 * GEOMETRY_REL_EPS
    return bary[1] >= -eps and bary[2] >= -eps and bary[3] >= -eps
end

--- Compute barycentric coordinates of a point with respect to a triangle.
--- @param self Vector
--- @param T Matrix
--- @return Vector|nil
function Geometry.hpoint_triangle_barycentric(self, T)
    local A = Vector:_new(T[1])
    local B = Vector:_new(T[2])
    local C = Vector:_new(T[3])

    local v0 = C:_hsub(A)
    local v1 = B:_hsub(A)
    local v2 = self:_hsub(A)

    local d00 = v0:_hinner(v0)
    local d01 = v0:_hinner(v1)
    local d11 = v1:_hinner(v1)
    local d20 = v2:_hinner(v0)
    local d21 = v2:_hinner(v1)
    local denom = d00 * d11 - d01 * d01

    local edge_scale = simplex_length_scale(T)
    local denom_epsilon = GEOMETRY_REL_EPS * edge_scale^4
    if math.abs(denom) <= denom_epsilon then
        return nil
    end

    local c_weight = (d11 * d20 - d01 * d21) / denom
    local b_weight = (d00 * d21 - d01 * d20) / denom
    local a_weight = 1 - b_weight - c_weight
    return Vector:_new{a_weight, b_weight, c_weight}
end

--- Reconstruct a point from barycentric coordinates on a triangle.
--- @param T Matrix
--- @param bary Vector
--- @return Vector
function Geometry.hpoint_from_triangle_barycentric(T, bary)
    local A = Vector:_new(T[1]):_scale(bary[1])
    local B = Vector:_new(T[2]):_scale(bary[2])
    local C = Vector:_new(T[3]):_scale(bary[3])
    return A:_add(B):_add(C)
end

--- Compute the parameter interval of a line segment lying inside a triangle.
--- The segment is P(t)=(1-t)P0+tP1 for 0<=t<=1.
--- @param self Matrix 2-row line segment
--- @param T Matrix triangle
--- @return number|nil t_min
--- @return number|nil t_max
function Geometry.hclip_line_segment_to_triangle_interval(self, T)
    local P0 = Vector:_new(self[1])
    local P1 = Vector:_new(self[2])
    local bary0 = Geometry.hpoint_triangle_barycentric(P0, T)
    local bary1 = Geometry.hpoint_triangle_barycentric(P1, T)
    local eps = 1e-12

    if bary0 == nil or bary1 == nil then
        return nil, nil
    end

    local t_min = 0
    local t_max = 1

    for i = 1, 3 do
        local start_val = bary0[i]
        local delta = bary1[i] - bary0[i]

        if math.abs(delta) < eps then
            if start_val < -3 * eps then
                return nil, nil
            end
        else
            local boundary_t = (-3 * eps - start_val) / delta
            if delta > 0 then
                if boundary_t > t_min then t_min = boundary_t end
            else
                if boundary_t < t_max then t_max = boundary_t end
            end
            if t_min > t_max + eps then
                return nil, nil
            end
        end
    end

    if t_max < 0 or t_min > 1 then
        return nil, nil
    end

    t_min = math.max(0, t_min)
    t_max = math.min(1, t_max)
    if t_max - t_min < eps then
        return nil, nil
    end

    return t_min, t_max
end

--- Clip a line segment to a triangle using barycentric half-spaces.
--- @param self Matrix
--- @param T Matrix
--- @return Matrix|nil
function Geometry.hclip_line_segment_to_triangle(self, T)
    local t_min, t_max = Geometry.hclip_line_segment_to_triangle_interval(self, T)
    if t_min == nil then
        return nil
    end

    local P0 = Vector:_new(self[1])
    local P1 = Vector:_new(self[2])
    local Q0 = P0:_scale(1 - t_min):_add(P1:_scale(t_min))
    local Q1 = P0:_scale(1 - t_max):_add(P1:_scale(t_max))
    if Q0:_hdistance(Q1) <= point_epsilon(GEOMETRY_REL_EPS, P0, P1, Q0, Q1) then
        return nil
    end

    return Matrix:_new{Q0, Q1}
end


local function copy_simplex_metadata_for_part(source, part_simplex)
    local meta = {}
    for k, v in pairs(source) do
        if k ~= "simplex" and k ~= "type" and k ~= "bbox2" then
            meta[k] = v
        end
    end
    return meta
end

local function normalize_partition_parts(parts, expected_rows)
    if type(parts) ~= "table" then
        return nil
    end

    local function is_degenerate_part(part)
        if expected_rows == 2 then
            local A = Vector:_new(part[1])
            local B = Vector:_new(part[2])
            return A:_hdistance(B) <= simplex_length_epsilon(part, PARTITION_REL_EPS)
        end

        if expected_rows == 3 then
            local A = Vector:_new(part[1])
            local B = Vector:_new(part[2])
            local C = Vector:_new(part[3])
            local area_twice = (B:_hsub(A)):_hcross(C:_hsub(A)):hnorm()
            return area_twice <= triangle_area_epsilon(part, PARTITION_REL_EPS)
        end

        return false
    end

    local indices = {}
    for key, _ in pairs(parts) do
        if type(key) == "number" and key >= 1 and key % 1 == 0 then
            indices[#indices + 1] = key
        end
    end

    if #indices == 0 then
        return nil
    end

    table.sort(indices)

    local normalized = {}
    for _, index in ipairs(indices) do
        local part = parts[index]
        if part ~= nil and getmetatable(part) == Matrix then
            if expected_rows == nil or #part == expected_rows then
                if expected_rows == nil or not is_degenerate_part(part) then
                    normalized[#normalized + 1] = part
                end
            end
        end
    end

    if #normalized == 0 then
        return nil
    end

    return normalized
end

--- Check if a homogeneous point intersects a homogeneous triangle
--- @param self Vector
--- @param T Matrix
--- @return boolean
function Geometry.hpoint_triangle_intersecting(self, T)
    local Tbasis = T:copy():hto_basis()
    local S = self:_horthogonal_projection_onto_plane(Tbasis)
    return Geometry.hpoint_in_triangular_prism(S, T)
       and Geometry.hpoint_point_intersecting(self, S)
end

--- Check occlusion sort of a homogeneous point with a homogeneous point
--- @param self Vector
--- @param P Vector
--- @return boolean|nil
function Geometry.hpoint_point_occlusion_sort(self, P)
    if not Geometry.hpoint_point_intersecting(self, P) then
        if Geometry.hpoint_point_intersecting(
            Vector:_new{self[1], self[2], 0, 1},
            Vector:_new{P[1], P[2], 0, 1}
        ) then
            return self[3] < P[3]
        end
    end
    return nil
end

--- Check collinearity of two homogeneous vectors
--- @param self Vector
--- @param other Vector
--- @return boolean
function Geometry.hcollinear(self, other)
    local cross = self:_hcross(other)
    local scale = math.max(
        self:hnorm() * other:hnorm(),
        MIN_GEOMETRY_SCALE * MIN_GEOMETRY_SCALE
    )
    return cross:hnorm() <= GEOMETRY_REL_EPS * scale
end

--- Check opposite direction of two homogeneous vectors
--- @param self Vector
--- @param other Vector
--- @return boolean
function Geometry.hoppositely_directed(self, other)
    return self:_hinner(other) < 0 and Geometry.hcollinear(self, other)
end

--- Check occlusion sort of a homogeneous point with a homogeneous line segment
--- @param self Vector
--- @param L Matrix
--- @return boolean|nil
function Geometry.hpoint_line_segment_occlusion_sort(self, L)
    local LA = L:hto_basis()
    local LO = Vector:_new(LA[1])
    local OP = self:_hsub(LO)
    local LU = Vector:_new(LA[2])
    local proj = OP:_hproject_onto(LU)
    local true_proj = LO:_hadd(proj)
    local F = Vector:_new{self[1], self[2], 0, 1}
    local G = Vector:_new{true_proj[1], true_proj[2], 0, 1}
    if Geometry.hpoint_point_intersecting(F, G) then
        local temp = 1
        if Geometry.hoppositely_directed(proj, LU) then
            temp = -1
        end
        local test = temp * proj:hnorm() / LU:hnorm()
        local eps = 1e-12
        if (eps < test and test < 1 - eps) then
            return Geometry.hpoint_point_occlusion_sort(self, true_proj)
        end
    end
    return nil
end

--- Check occlusion sort of a homogeneous point with a homogeneous triangle
--- @param self Vector
--- @param T Matrix
--- @return boolean|nil
function Geometry.hpoint_triangle_occlusion_sort(self, T)
    local P1 = Vector:_new{self[1], self[2], 0, 1}
    local T1 = Vector:_new{T[1][1], T[1][2], 0, 1}
    local T2 = Vector:_new{T[2][1], T[2][2], 0, 1}
    local T3 = Vector:_new{T[3][1], T[3][2], 0, 1}
    local TP = Matrix:_new{
        Vector:_new{T1[1], T1[2], 0, 1},
        Vector:_new{T2[1], T2[2], 0, 1},
        Vector:_new{T3[1], T3[2], 0, 1}
    }
    local Ta = Vector:_new(T[1])
    local Tb = Vector:_new(T[2])
    local Tc = Vector:_new(T[3])

    local vertex_eps = point_epsilon(GEOMETRY_REL_EPS, self, Ta, Tb, Tc)
    if self:_hdistance(Ta) <= vertex_eps then return nil end
    if self:_hdistance(Tb) <= vertex_eps then return nil end
    if self:_hdistance(Tc) <= vertex_eps then return nil end
    local eps = 1e-12
    if Geometry.hpoint_in_triangular_prism(P1, TP) then
        local vu = Tb:_hsub(Ta)
        local vv = Tc:_hsub(Ta)
        local sol = Matrix:_new{
            Vector:_new{vu[1], vu[2]},
            Vector:_new{vv[1], vv[2]},
            Vector:_new{P1:_hsub(Ta)[1], P1:_hsub(Ta)[2]}
        }:column_reduction()
        local t, s
        if sol then
            t, s = sol[1], sol[2]
        else return nil end
        if (
            -eps < t and t < 1 + eps
            and -eps < s and s < 1 + eps
        ) then
            local a = Ta:_hadd(vu:_hscale(t)):_hadd(vv:_hscale(s))
            return Geometry.hpoint_point_occlusion_sort(self, a)
        else
            return nil
        end
    end
    return nil
end

--- Unified line-segment / point intersection test.
--- @param self Matrix  2-row line segment
--- @param point Vector The point to test
--- @param boundary_inclusive boolean If true, endpoints count as intersecting
--- @return boolean
function Geometry.hline_segment_point_intersecting(self, point, boundary_inclusive)
    local LA = self:hto_basis()
    local LO = Vector:_new(LA[1])
    local LU = Vector:_new(LA[2])
    local LOP = point:_hsub(LO)
    local rhs = LOP:_hproject_onto(LU)
    local orth = LO:_hadd(rhs)

    if not Geometry.hpoint_point_intersecting(orth, point) then
        return false
    end

    local sol = Matrix:_new{
        Vector:_new{LU[1], LU[2], LU[3]},
        Vector:_new{0, 0, 0},
        Vector:_new{0, 0, 0},
        Vector:_new{rhs[1], rhs[2], rhs[3]}
    }:column_reduction()
    if sol == nil then return false end
    local t = sol[1]

    local eps = 1e-12
    if boundary_inclusive then
        return -eps < t and t < 1 + eps
    else
        return eps < t and t < 1 - eps
    end
end

--- Partition a line segment by a point
--- @param self Matrix
--- @param point Vector
--- @return table|nil
function Geometry.hpartition_line_segment_by_point(self, point)
    if Geometry.hline_segment_point_intersecting(self, point, false) then
        return {
            Matrix:_new{self[1], point},
            Matrix:_new{point, self[2]}
        }
    end
    return nil
end

--- Compute the intersection of two lines (basis form)
--- @param self Matrix
--- @param line Matrix
--- @return Vector|nil
function Geometry.hline_line_intersection(self, line)
    local O1 = Vector:_new(self[1])
    local U1 = Vector:_new(self[2])
    local O2 = Vector:_new(line[1])
    local U2 = Vector:_new(line[2])
    local rhs = O2:_hsub(O1)
    local sol = Matrix:_new{
        Vector:_new{U1[1], U1[2], U1[3]},
        Vector:_new{-U2[1], -U2[2], -U2[3]},
        Vector:_new{0, 0, 0},
        Vector:_new{rhs[1], rhs[2], rhs[3]}
    }:column_reduction()
    if sol == nil then return nil end
    return O1:_hadd(U1:_hscale(sol[1]))
end

--- Compute the intersection of two line segments
--- @param self Matrix
--- @param line Matrix
--- @return Vector|nil
function Geometry.hline_segment_line_segment_intersection(self, line)
    local L1A = self:hto_basis()
    local L2A = line:hto_basis()
    local I = Geometry.hline_line_intersection(L1A, L2A)
    if I == nil then return nil end
    if Geometry.hline_segment_point_intersecting(self, I, false)
    and Geometry.hline_segment_point_intersecting(line, I, false) then
        return I
    end
    return nil
end

--- Compute the intersection of a line and a plane (both in basis form)
--- @param self Matrix line basis
--- @param plane Matrix plane basis
--- @return Vector|nil
function Geometry.hline_plane_intersection(self, plane)
    local LO = Vector:_new(self[1])
    local LU = Vector:_new(self[2])
    local TO = Vector:_new(plane[1])
    local TU = Vector:_new(plane[2])
    local TV = Vector:_new(plane[3])
    local rhs = TO:_hsub(LO)
    local sol = Matrix:_new{
        Vector:_new{LU[1], LU[2], LU[3]},
        Vector:_new{-TU[1], -TU[2], -TU[3]},
        Vector:_new{-TV[1], -TV[2], -TV[3]},
        Vector:_new{rhs[1], rhs[2], rhs[3]}
    }:column_reduction()
    if sol == nil then return nil end
    return LO:_hadd(LU:_hscale(sol[1]))
end

--- Compute the intersection of a line segment and a triangle
--- @param self Matrix line segment (2 rows)
--- @param tri Matrix triangle (3 rows)
--- @return Vector|nil
function Geometry.hline_segment_triangle_intersection(self, tri)
    local coincident_points = 0
    for _, P1 in ipairs(self) do
        for _, P2 in ipairs(tri) do
            if Geometry.hpoint_point_intersecting(Vector:_new(P1), Vector:_new(P2)) then
                coincident_points = coincident_points + 1
            end
        end
    end
    if coincident_points > 1 then
        return nil
    end
    local I = Geometry.hline_plane_intersection(self:hto_basis(), tri:hto_basis())
    if I == nil then return nil end
    if not Geometry.hline_segment_point_intersecting(self, I, true) then
        return nil
    end
    if not Geometry.hpoint_in_triangular_prism(I, tri) then
        return nil
    end
    return I
end

--- Compute the intersection of two triangles
--- @param self Matrix
--- @param tri Matrix
--- @return Matrix|nil  2-row intersection segment, or nil
function Geometry.htriangle_triangle_intersections(self, tri)
    local edges1 = {
        Matrix:_new{self[1], self[2]},
        Matrix:_new{self[2], self[3]},
        Matrix:_new{self[3], self[1]}
    }
    local edges2 = {
        Matrix:_new{tri[1], tri[2]},
        Matrix:_new{tri[2], tri[3]},
        Matrix:_new{tri[3], tri[1]}
    }
    local points = {}
    local merge_eps = MERGE_REL_EPS * math.max(
        simplex_length_scale(self),
        simplex_length_scale(tri)
    )

    local function same_point(p1, p2)
        return p1:_hdistance(p2) < merge_eps
    end

    local function add_unique(point)
        for _, p in ipairs(points) do
            if same_point(point, p) then
                return nil
            end
        end
        table.insert(points, point)
    end

    local function point_in_triangle_tol(point, triangle)
        local A = Vector:_new(triangle[1])
        local B = Vector:_new(triangle[2])
        local C = Vector:_new(triangle[3])

        local v0 = C:_hsub(A)
        local v1 = B:_hsub(A)
        local v2 = point:_hsub(A)

        local d00 = v0:_hinner(v0)
        local d01 = v0:_hinner(v1)
        local d11 = v1:_hinner(v1)
        local d20 = v2:_hinner(v0)
        local d21 = v2:_hinner(v1)
        local denom = d00 * d11 - d01 * d01
        if math.abs(denom) < merge_eps then
            return false
        end

        local v = (d11 * d20 - d01 * d21) / denom
        local w = (d00 * d21 - d01 * d20) / denom
        local u = 1 - v - w
        return u >= -3 * merge_eps and v >= -3 * merge_eps and w >= -3 * merge_eps
    end

    local function line_contains_point(edge, point)
        local LA = edge:hto_basis()
        local LO = LA[1]
        local LU = LA[2]
        local LOP = point:_hsub(LO)
        local rhs = LOP:_hproject_onto(LU)
        local orth = LO:_hadd(rhs)

        if not same_point(orth, point) then
            return false
        end

        local sol = Matrix:_new{
            Vector:_new{LU[1], LU[2], LU[3]},
            Vector:_new{0, 0, 0},
            Vector:_new{0, 0, 0},
            Vector:_new{rhs[1], rhs[2], rhs[3]}
        }:column_reduction()
        if sol == nil then return false end

        local t = sol[1]
        return -merge_eps < t and t < 1 + merge_eps
    end

    local function edge_triangle_point(edge, triangle)
        local I = Geometry.hline_plane_intersection(edge:hto_basis(), triangle:hto_basis())
        if I == nil then return nil end
        if not line_contains_point(edge, I) then return nil end
        if not point_in_triangle_tol(I, triangle) then return nil end
        return I
    end

    for _, edge in ipairs(edges1) do
        local I = edge_triangle_point(edge, tri)
        if I ~= nil then
            add_unique(I)
        end
    end
    for _, edge in ipairs(edges2) do
        local I = edge_triangle_point(edge, self)
        if I ~= nil then
            add_unique(I)
        end
    end

    if #points < 2 then
        return nil
    end

    if #points > 2 then
        local origin = points[1]
        local direction = nil

        for i = 2, #points do
            local diff = points[i]:_hsub(origin)
            if diff:hnorm() > merge_eps then
                direction = diff
                break
            end
        end

        if direction == nil then
            return nil
        end

        local function are_collinear(a, b)
            return a:_hcross(b):hnorm() < merge_eps
        end

        for i = 2, #points do
            local diff = points[i]:_hsub(origin)
            if diff:hnorm() > merge_eps and not are_collinear(direction, diff) then
                return nil
            end
        end

        local min_point = points[1]
        local max_point = points[1]
        local min_t = 0
        local max_t = 0
        for i = 2, #points do
            local diff = points[i]:_hsub(origin)
            local t = diff:hnorm()
            if diff:_hinner(direction) < 0 then
                t = -t
            end
            if t < min_t then
                min_t = t
                min_point = points[i]
            end
            if t > max_t then
                max_t = t
                max_point = points[i]
            end
        end

        if same_point(min_point, max_point) then
            return nil
        end
        points = { min_point, max_point }
    end

    if #points ~= 2 then
        return nil
    end

    local ans = {}
    for _, p in ipairs(points) do
        table.insert(ans, p)
    end
    return Matrix:_new(ans)
end

--- Partition a line segment by another line segment
--- @param self Matrix
--- @param line Matrix
--- @return table|nil
function Geometry.hpartition_line_segment_by_line_segment(self, line)
    local I = Geometry.hline_segment_line_segment_intersection(self, line)
    if I ~= nil then
        return Geometry.hpartition_line_segment_by_point(self, I)
    end
    return nil
end

--- Partition a line segment by a triangle
--- @param self Matrix
--- @param tri Matrix
--- @return table|nil
function Geometry.hpartition_line_segment_by_triangle(self, tri)
    local I = Geometry.hline_segment_triangle_intersection(self, tri)
    if I ~= nil then
        return Geometry.hpartition_line_segment_by_point(self, I)
    end
    return nil
end


--- Partition a triangle by another triangle
--- @param self Matrix
--- @param tri Matrix
--- @return table|nil  list of sub-triangle Matrices
function Geometry.hpartition_triangle_by_triangle(self, tri)
    local I = Geometry.htriangle_triangle_intersections(self, tri)
    if I == nil then return nil end

    local merge_eps = MERGE_REL_EPS * math.max(
        simplex_length_scale(self),
        simplex_length_scale(tri)
    )
    local tri_basis = tri:hto_basis()
    local edges1 = {
        Matrix:_new{self[1], self[2]},
        Matrix:_new{self[2], self[3]},
        Matrix:_new{self[3], self[1]}
    }

    -- Find where each edge of self crosses the PLANE of tri
    local hits = {}  -- each: {point=Vector, edges={[i]=true}}
    for i, edge in ipairs(edges1) do
        local int = Geometry.hline_plane_intersection(edge:hto_basis(), tri_basis)
        if int ~= nil and Geometry.hline_segment_point_intersecting(edges1[i], int, true) then
            local merged = false
            for _, h in ipairs(hits) do
                if h.point:_hdistance(int) < merge_eps then
                    h.edges[i] = true
                    merged = true
                    break
                end
            end
            if not merged then
                table.insert(hits, {point = int, edges = {[i] = true}})
            end
        end
    end

    if #hits ~= 2 then
        return nil
    end

    local vertices = {
        Vector:_new(self[1]),
        Vector:_new(self[2]),
        Vector:_new(self[3])
    }
    local A, B = hits[1].point, hits[2].point

    -- Vertex detection with tolerant merge
    -- V1 is shared by edges {1,3}, V2 by {1,2}, V3 by {2,3}
    local a_vertex = nil
    local b_vertex = nil
    for i = 1, 3 do
        if A:_hdistance(vertices[i]) < merge_eps then a_vertex = i end
        if B:_hdistance(vertices[i]) < merge_eps then b_vertex = i end
    end

    if a_vertex ~= nil and b_vertex ~= nil then
        return nil
    end

    if a_vertex ~= nil or b_vertex ~= nil then
        local vertex_index = a_vertex or b_vertex
        local edge_point = a_vertex and B or A
        local next_index = (vertex_index % 3) + 1
        local prev_index = ((vertex_index + 1) % 3) + 1
        return {
            Matrix:_new{
                vertices[vertex_index],
                vertices[next_index],
                edge_point
            },
            Matrix:_new{
                vertices[vertex_index],
                edge_point,
                vertices[prev_index]
            },
            Matrix:_new{
                vertices[next_index],
                edge_point,
                vertices[prev_index]
            }
        }
    end

    -- Neither point is a vertex: find the non-intersecting edge from tracking
    local edge_has_point = {false, false, false}
    for _, h in ipairs(hits) do
        for i = 1, 3 do
            if h.edges[i] then edge_has_point[i] = true end
        end
    end

    local non_intersecting = nil
    for i = 1, 3 do
        if not edge_has_point[i] then
            non_intersecting = i
            break
        end
    end

    if non_intersecting == nil then
        return nil
    end

    local quad = {}
    local tri1
    table.insert(quad, A)
    table.insert(quad, B)
    if non_intersecting == 1 then
        table.insert(quad, edges1[1][1])
        table.insert(quad, edges1[1][2])
        tri1 = Matrix:_new{A, B, edges1[2][2]}
    elseif non_intersecting == 2 then
        table.insert(quad, edges1[2][1])
        table.insert(quad, edges1[2][2])
        tri1 = Matrix:_new{A, B, edges1[3][2]}
    elseif non_intersecting == 3 then
        table.insert(quad, edges1[3][1])
        table.insert(quad, edges1[3][2])
        tri1 = Matrix:_new{A, B, edges1[1][2]}
    end
    quad = Matrix:_new(quad):hcentroid_sort()
    local Q1 = Vector:_new(quad[1])
    local Q2 = Vector:_new(quad[2])
    local Q3 = Vector:_new(quad[3])
    local Q4 = Vector:_new(quad[4])
    if Q1:_hdistance(Q3) > Q2:_hdistance(Q4) then
        return {
            tri1,
            Matrix:_new{Q2, Q1, Q4},
            Matrix:_new{Q2, Q3, Q4}
        }
    else
        return {
            tri1,
            Matrix:_new{Q1, Q2, Q3},
            Matrix:_new{Q3, Q4, Q1}
        }
    end
end

--- Partition a triangle by vertical edge-planes of another triangle.
--- @param self Matrix
--- @param tri Matrix
--- @return table|nil
function Geometry.hpartition_triangle_by_edge_planes(self, tri)
    local z_min = math.min(self[1][3], self[2][3], self[3][3])
    local z_max = math.max(self[1][3], self[2][3], self[3][3])
    local coordinate_margin = point_epsilon(
        PARTITION_REL_EPS,
        self[1], self[2], self[3], tri[1], tri[2], tri[3]
    )
    local z_margin = math.max((z_max - z_min) * 0.1, coordinate_margin)
    z_min = z_min - z_margin
    z_max = z_max + z_margin

    local cutting_tris = {}
    local edges = {
        {tri[1], tri[2]},
        {tri[2], tri[3]},
        {tri[3], tri[1]}
    }

    for _, edge in ipairs(edges) do
        local A = edge[1]
        local B = edge[2]
        local dx = A[1] - B[1]
        local dy = A[2] - B[2]
        local xy_scale = math.max(
            MIN_GEOMETRY_SCALE,
            math.abs(A[1]), math.abs(A[2]),
            math.abs(B[1]), math.abs(B[2])
        )
        if math.sqrt(dx * dx + dy * dy) <= GEOMETRY_REL_EPS * xy_scale then
            goto continue_edge
        end
        local BL = Vector:_new{A[1], A[2], z_min, 1}
        local BR = Vector:_new{B[1], B[2], z_min, 1}
        local TR = Vector:_new{B[1], B[2], z_max, 1}
        local TL = Vector:_new{A[1], A[2], z_max, 1}
        table.insert(cutting_tris, Matrix:_new{BL, BR, TR})
        table.insert(cutting_tris, Matrix:_new{BL, TR, TL})
        ::continue_edge::
    end

    local pieces = {self:copy()}
    for _, cutter in ipairs(cutting_tris) do
        local new_pieces = {}
        for _, piece in ipairs(pieces) do
            local parts = Geometry.hpartition_triangle_by_triangle(piece, cutter)
            local normalized = normalize_partition_parts(parts, 3)
            if normalized ~= nil then
                for _, part in ipairs(normalized) do
                    table.insert(new_pieces, part)
                end
            else
                table.insert(new_pieces, piece)
            end
        end
        pieces = new_pieces
    end

    if #pieces > 1 then
        return pieces
    end
    return nil
end


--- line-segment vs line-segment occlusion
function Geometry.hline_segment_line_segment_occlusion_sort(self, L)
    local L1A = Vector:_new{self[1][1], self[1][2], 0, 1}
    local L1B = Vector:_new{self[2][1], self[2][2], 0, 1}
    local L2A = Vector:_new{L[1][1], L[1][2], 0, 1}
    local L2B = Vector:_new{L[2][1], L[2][2], 0, 1}
    local L1_dir = L1B:_hsub(L1A)
    local L2_dir = L2B:_hsub(L2A)

    local RL1A = Vector:_new(self[1])
    local RL1B = Vector:_new(self[2])
    local RL2A = Vector:_new(L[1])
    local RL2B = Vector:_new(L[2])

    local eps = 1e-12
    if not Geometry.hcollinear(L1_dir, L2_dir) then
        local sol = Matrix:_new{
            Vector:_new{L1_dir[1], L1_dir[2]},
            Vector:_new{-L2_dir[1], -L2_dir[2]},
            Vector:_new{L2A[1] - L1A[1], L2A[2] - L1A[2]}
        }:column_reduction()
        local t, s
        if sol ~= nil then
            t, s = sol[1], sol[2]
        else return nil end
        if (eps < t and t < 1 - eps and eps < s and s < 1 - eps) then
            local RL1I = RL1A:_hadd((RL1B:_hsub(RL1A)):_hscale(t))
            local RL2I = RL2A:_hadd((RL2B:_hsub(RL2A)):_hscale(s))
            return Geometry.hpoint_point_occlusion_sort(RL1I, RL2I)
        end
    else
        local M1 = Geometry.hpoint_line_segment_occlusion_sort(RL1A, L)
        if M1 ~= nil then return M1 end
        local M2 = Geometry.hpoint_line_segment_occlusion_sort(RL1B, L)
        if M2 ~= nil then return M2 end
        local M3 = Geometry.hpoint_line_segment_occlusion_sort(RL2A, self)
        if M3 ~= nil then return not M3 end
        local M4 = Geometry.hpoint_line_segment_occlusion_sort(RL2B, self)
        if M4 ~= nil then return not M4 end
    end
    return nil
end

--- line-segment vs triangle occlusion
function Geometry.hline_segment_triangle_occlusion_sort(self, T)
    local points1 = { Vector:_new(self[1]), Vector:_new(self[2]) }
    for _, p1 in ipairs(points1) do
        local A = Geometry.hpoint_triangle_occlusion_sort(p1, T)
        if A ~= nil then return A end
    end
    local edges2 = {
        Matrix:_new{T[1], T[2]},
        Matrix:_new{T[2], T[3]},
        Matrix:_new{T[3], T[1]}
    }
    for _, e2 in ipairs(edges2) do
        local A = Geometry.hline_segment_line_segment_occlusion_sort(self, e2)
        if A ~= nil then return A end
    end
    return nil
end

--- triangle vs triangle occlusion
function Geometry.htriangle_triangle_occlusion_sort(self, T)
    local edges1 = {
        Matrix:_new{self[1], self[2]},
        Matrix:_new{self[2], self[3]},
        Matrix:_new{self[3], self[1]}
    }
    for _, e1 in ipairs(edges1) do
        local A = Geometry.hline_segment_triangle_occlusion_sort(e1, T)
        if A ~= nil then return A end
    end
    local points1 = { Vector:_new(self[1]), Vector:_new(self[2]), Vector:_new(self[3]) }
    for _, p1 in ipairs(points1) do
        local A = Geometry.hpoint_triangle_occlusion_sort(p1, T)
        if A ~= nil then return A end
    end
    local points2 = { Vector:_new(T[1]), Vector:_new(T[2]), Vector:_new(T[3]) }
    for _, p2 in ipairs(points2) do
        local A = Geometry.hpoint_triangle_occlusion_sort(p2, self)
        if A ~= nil then return not A end
    end
    return nil
end


--- Return the required ordering for a surface patch and a curve fragment it
--- supports.  true means S1 must be rendered before S2; false means S2 must
--- be rendered before S1.  nil means there is no support relationship.
--- Support is deliberately local: every non-supporting simplex still goes
--- through the ordinary occlusion predicates.
function Geometry.support_order(S1, S2)
    if S1.surface_patch_id ~= nil
        and S2.support_patch_ids ~= nil
        and S2.support_patch_ids[S1.surface_patch_id]
    then
        return true
    end

    if S2.surface_patch_id ~= nil
        and S1.support_patch_ids ~= nil
        and S1.support_patch_ids[S2.surface_patch_id]
    then
        return false
    end

    return nil
end

--- Top-level occlusion sort dispatcher
--- @param S1 table simplex record
--- @param S2 table simplex record
--- @return boolean|nil
function Geometry.occlusion_sort_simplices(S1, S2)
    if S1.type == "point" and S2.type == "point" then
        return Geometry.hpoint_point_occlusion_sort(S1.simplex, S2.simplex)
    elseif S1.type == "point" and S2.type == "line segment" then
        return Geometry.hpoint_line_segment_occlusion_sort(S1.simplex, S2.simplex)
    elseif S1.type == "line segment" and S2.type == "point" then
        local A = Geometry.hpoint_line_segment_occlusion_sort(S2.simplex, S1.simplex)
        if A ~= nil then return not A end
    elseif S1.type == "point" and S2.type == "triangle" then
        return Geometry.hpoint_triangle_occlusion_sort(S1.simplex, S2.simplex)
    elseif S1.type == "triangle" and S2.type == "point" then
        local A = Geometry.hpoint_triangle_occlusion_sort(S2.simplex, S1.simplex)
        if A ~= nil then return not A end
    elseif S1.type == "line segment" and S2.type == "line segment" then
        return Geometry.hline_segment_line_segment_occlusion_sort(S1.simplex, S2.simplex)
    elseif S1.type == "line segment" and S2.type == "triangle" then
        local A = Geometry.hline_segment_triangle_occlusion_sort(S1.simplex, S2.simplex)
        if A == nil then return nil else return A end
    elseif S1.type == "triangle" and S2.type == "line segment" then
        local A = Geometry.hline_segment_triangle_occlusion_sort(S2.simplex, S1.simplex)
        if A == nil then return nil else return not A end
    elseif S1.type == "triangle" and S2.type == "triangle" then
        return Geometry.htriangle_triangle_occlusion_sort(S1.simplex, S2.simplex)
    end
end

--- Strongly connected components sort for simplices based on occlusion.
--- Uses a 2D AABB hierarchy to reduce pair comparisons, Tarjan's SCC to detect cycles,
--- and edge-plane partitioning to break them.
--- @param simplices table list of simplex records
--- @param depth number|nil
--- @return table sorted list
--- @return table diagnostics
function Geometry.scc(simplices, depth)
    depth = depth or 0
    local max_depth = 16
    local n = #simplices

    local diagnostics = {
        depth = depth,
        had_cycle = false,
        unresolved_cycles = false,
        unresolved_component_count = 0,
        max_depth_reached = false,
        candidate_pairs = 0,
        exact_occlusion_tests = 0,
        graph_edges = 0,
        support_edges = 0,
        scc_components = 0,
        cyclic_components = 0,
        cycle_pair_subdivisions = 0,
        cycle_temporary_pieces = 0,
    }

    -- Occlusion can only occur between simplices whose projected 2D AABBs
    -- overlap.  The BVH supplies each such candidate pair exactly once; the
    -- existing exact occlusion predicates remain authoritative.
    local occlusion_entries = {}
    for i, simplex in ipairs(simplices) do
        if simplex.type ~= "label" and simplex.type ~= "point" then
            simplex.bbox2 = simplex.bbox2 or simplex.simplex:get_bbox2()
            occlusion_entries[#occlusion_entries + 1] = {
                index = i,
                box = simplex.bbox2,
            }
        end
    end
    local occlusion_tree = AABBTree.build(occlusion_entries, 2)

    local adj = {}
    local adj_set = {}
    for i = 1, n do
        adj[i] = {}
        adj_set[i] = {}
    end

    occlusion_tree:foreach_overlapping_pair(function(i, j)
        diagnostics.candidate_pairs = diagnostics.candidate_pairs + 1
        local si, sj = simplices[i], simplices[j]
        -- Keep this exact bbox predicate as a defensive invariant check even
        -- though the BVH has already established the same conservative test.
        if si.simplex:bboxes_overlap2(sj.simplex, si.bbox2, sj.bbox2) then
            local support_cmp = Geometry.support_order(si, sj)
            if support_cmp == true then
                adj[i][#adj[i] + 1] = j
                adj_set[i][j] = true
                diagnostics.graph_edges = diagnostics.graph_edges + 1
                diagnostics.support_edges = diagnostics.support_edges + 1
            elseif support_cmp == false then
                adj[j][#adj[j] + 1] = i
                adj_set[j][i] = true
                diagnostics.graph_edges = diagnostics.graph_edges + 1
                diagnostics.support_edges = diagnostics.support_edges + 1
            else
                diagnostics.exact_occlusion_tests = diagnostics.exact_occlusion_tests + 1
                local cmp = Geometry.occlusion_sort_simplices(si, sj)
                if cmp == true then
                    adj[i][#adj[i] + 1] = j
                    adj_set[i][j] = true
                    diagnostics.graph_edges = diagnostics.graph_edges + 1
                elseif cmp == false then
                    adj[j][#adj[j] + 1] = i
                    adj_set[j][i] = true
                    diagnostics.graph_edges = diagnostics.graph_edges + 1
                end
            end
        end
    end)

    -- Graph algorithms must not depend on BVH traversal order.
    for i = 1, n do
        table.sort(adj[i])
    end

    -- Tarjan's SCC
    local index_counter = 0
    local stack = {}
    local on_stack = {}
    local indices = {}
    local lowlinks = {}
    local components = {}

    local function strongconnect(v)
        indices[v] = index_counter
        lowlinks[v] = index_counter
        index_counter = index_counter + 1
        table.insert(stack, v)
        on_stack[v] = true

        for _, w in ipairs(adj[v]) do
            if indices[w] == nil then
                strongconnect(w)
                lowlinks[v] = math.min(lowlinks[v], lowlinks[w])
            elseif on_stack[w] then
                lowlinks[v] = math.min(lowlinks[v], indices[w])
            end
        end

        if lowlinks[v] == indices[v] then
            local component = {}
            while true do
                local w = table.remove(stack)
                on_stack[w] = false
                table.insert(component, w)
                if w == v then break end
            end
            table.insert(components, component)
        end
    end

    for i = 1, n do
        if indices[i] == nil then strongconnect(i) end
    end

    -- Cycle resolution
    local has_cycle = false
    local cycle_component_count = 0
    for _, comp in ipairs(components) do
        if #comp > 1 then
            has_cycle = true
            cycle_component_count = cycle_component_count + 1
        end
    end

    diagnostics.had_cycle = has_cycle
    diagnostics.scc_components = #components
    diagnostics.cyclic_components = cycle_component_count

    -- Cycles are a rendering-order problem, so subdivisions made solely to
    -- break a cycle are temporary render fragments.  For each cyclic SCC,
    -- choose one triangle pair and subdivide only those two triangles by one
    -- another's projected edge planes.  All other members of the SCC remain
    -- untouched during this resolution step.
    --
    -- The exact edge-plane partition routine is unchanged.  This block only
    -- narrows the lifetime and scope of the fragments it produces.
    if has_cycle and depth < max_depth then
        local replacements = {}

        local function temporary_cycle_parts(source, cutter)
            local parts = Geometry.hpartition_triangle_by_edge_planes(
                source.simplex,
                cutter.simplex
            )
            local normalized = normalize_partition_parts(parts, 3)
            if normalized == nil then
                return nil
            end

            local result = {}
            for _, part in ipairs(normalized) do
                local fragment = {
                    simplex = part,
                    type = "triangle",
                    bbox2 = part:get_bbox2(),
                }
                local meta = copy_simplex_metadata_for_part(source, part)
                for k, v in pairs(meta) do fragment[k] = v end
                result[#result + 1] = fragment
            end
            return result
        end

        for _, comp in ipairs(components) do
            if #comp > 1 then
                -- Tarjan's pop order is deterministic here, but sorting makes
                -- pair selection explicitly depend only on original indices.
                local members = {}
                for _, idx in ipairs(comp) do members[#members + 1] = idx end
                table.sort(members)

                local function directly_related(i, j)
                    return adj_set[i][j] == true or adj_set[j][i] == true
                end

                local pair_resolved = false
                for ai = 1, #members - 1 do
                    if pair_resolved then break end
                    local a_idx = members[ai]
                    local a = simplices[a_idx]
                    if a.type == "triangle" then
                        for bi = ai + 1, #members do
                            local b_idx = members[bi]
                            local b = simplices[b_idx]
                            if b.type == "triangle"
                                and directly_related(a_idx, b_idx)
                            then
                                -- Attempt the same subdivision idea in both
                                -- directions.  A direction that needs no split
                                -- simply keeps its original simplex.
                                local a_parts = temporary_cycle_parts(a, b)
                                local b_parts = temporary_cycle_parts(b, a)

                                if a_parts ~= nil or b_parts ~= nil then
                                    replacements[a_idx] = a_parts or {a}
                                    replacements[b_idx] = b_parts or {b}
                                    diagnostics.cycle_pair_subdivisions =
                                        diagnostics.cycle_pair_subdivisions + 1
                                    diagnostics.cycle_temporary_pieces =
                                        diagnostics.cycle_temporary_pieces
                                        + #(replacements[a_idx])
                                        + #(replacements[b_idx])
                                    pair_resolved = true
                                    break
                                end
                            end
                        end
                    end
                end
            end
        end

        if next(replacements) then
            local temporary_simplices = {}
            for i, simplex in ipairs(simplices) do
                local replacement = replacements[i]
                if replacement ~= nil then
                    for _, fragment in ipairs(replacement) do
                        temporary_simplices[#temporary_simplices + 1] = fragment
                    end
                else
                    temporary_simplices[#temporary_simplices + 1] = simplex
                end
            end

            local sorted, recursive_diagnostics = Geometry.scc(
                temporary_simplices,
                depth + 1
            )
            recursive_diagnostics = recursive_diagnostics or diagnostics
            recursive_diagnostics.had_cycle =
                recursive_diagnostics.had_cycle or has_cycle
            for _, key in ipairs({
                "candidate_pairs",
                "exact_occlusion_tests",
                "graph_edges",
                "support_edges",
                "scc_components",
                "cyclic_components",
                "cycle_pair_subdivisions",
                "cycle_temporary_pieces",
            }) do
                recursive_diagnostics[key] =
                    (recursive_diagnostics[key] or 0) + (diagnostics[key] or 0)
            end
            return sorted, recursive_diagnostics
        end
    end

    if has_cycle then
        diagnostics.unresolved_cycles = true
        diagnostics.unresolved_component_count = cycle_component_count
        diagnostics.max_depth_reached = depth >= max_depth

        local warning = "Unresolved occlusion cycle(s) remain after SCC sorting "
            .. "(%d component(s) at depth %d); rendering order may be unstable."
        log_line(warning:format(cycle_component_count, depth))
    end

    -- Topological sort (DFS)
    local visited = {}
    local sorted = {}

    local function visit(u)
        if visited[u] then return end
        visited[u] = true
        for _, v in ipairs(adj[u]) do
            visit(v)
        end
        sorted[#sorted + 1] = simplices[u]
    end

    for i = 1, n do visit(i) end

    -- DFS completion order is reversed once to obtain the required draw order
    -- without repeated front insertion into the Lua array.
    for i = 1, math.floor(#sorted / 2) do
        local j = #sorted - i + 1
        sorted[i], sorted[j] = sorted[j], sorted[i]
    end

    return sorted, diagnostics
end

--- Partition simplices by their parents recursively, retaining all terminal pieces.
--- Uses a 3D AABB tree to query only parents that can intersect each piece.
--- Skips self-partitioning.
--- @param simplices table
--- @param parents table
--- @return table terminal pieces
--- @return table diagnostics
function Geometry.partition_simplices_by_parents(simplices, parents)
    local result = {}
    local stats = {
        input_simplices = #simplices,
        indexed_parents = 0,
        candidate_queries = 0,
        candidate_parents = 0,
        exact_partition_tests = 0,
        successful_splits = 0,
        pieces_created = 0,
        max_depth = 0,
        depth_limit_hits = 0,
        output_pieces = 0,
    }

    -- Parent geometry is static throughout recursive subdivision.  Build one
    -- 3D BVH and use it only as a conservative broad phase; exact overlap and
    -- partition predicates below remain authoritative.
    local parent_bbox3 = {}
    local parent_entries = {}
    for pi, parent in ipairs(parents) do
        if parent.type ~= "point" and parent.type ~= "label" then
            local bbox3 = parent.simplex:get_bbox3()
            parent_bbox3[pi] = bbox3
            parent.bbox2 = parent.bbox2 or parent.simplex:get_bbox2()
            parent_entries[#parent_entries + 1] = {
                index = pi,
                box = bbox3,
            }
        end
    end

    local parent_tree = AABBTree.build(parent_entries, 3)
    stats.indexed_parents = #parent_entries

    local max_depth = 50

    local function partition_recursive(piece, depth, skip_parent)
        depth = depth or 0
        if depth > stats.max_depth then stats.max_depth = depth end
        if depth >= max_depth then
            stats.depth_limit_hits = stats.depth_limit_hits + 1
            table.insert(result, piece)
            return
        end
        if piece.type == "point" or piece.type == "label" then
            table.insert(result, piece)
            return
        end

        local meta = {}
        for k, v in pairs(piece) do
            if k ~= "simplex" and k ~= "type" and k ~= "bbox2" then
                meta[k] = v
            end
        end

        local split_occurred = false
        piece.bbox2 = piece.bbox2 or piece.simplex:get_bbox2()
        local piece_bbox2 = piece.bbox2
        local piece_bbox3 = piece.simplex:get_bbox3()
        local candidate_indices = {}
        stats.candidate_queries = stats.candidate_queries + 1
        parent_tree:query(piece_bbox3, function(pi)
            candidate_indices[#candidate_indices + 1] = pi
        end)
        stats.candidate_parents = stats.candidate_parents + #candidate_indices
        -- Preserve the original parent-index decision order independently of
        -- BVH traversal order.  This matters because the first successful
        -- partition determines the recursive branch taken next.
        table.sort(candidate_indices)

        for _, pi in ipairs(candidate_indices) do
            local parent = parents[pi]
            if pi ~= skip_parent
                and piece ~= parent
                and Geometry.support_order(piece, parent) == nil
            then
                local pb = parent_bbox3[pi]
                local parent_bbox2 = parent.bbox2
                if parent_bbox2
                    and piece.simplex:bboxes_overlap2(parent.simplex, piece_bbox2, parent_bbox2)
                    and pb
                    and piece.simplex:bboxes_overlap3(parent.simplex, piece_bbox3, pb)
                then
                    local parts = nil
                    local piece_type = piece.type
                    local parent_type = parent.type

                    if parent_type == "line segment" and piece_type == "line segment" then
                        stats.exact_partition_tests = stats.exact_partition_tests + 1
                        parts = Geometry.hpartition_line_segment_by_line_segment(piece.simplex, parent.simplex)
                    elseif parent_type == "triangle" and piece_type == "line segment" then
                        stats.exact_partition_tests = stats.exact_partition_tests + 1
                        parts = Geometry.hpartition_line_segment_by_triangle(piece.simplex, parent.simplex)
                    elseif parent_type == "triangle" and piece_type == "triangle" then
                        stats.exact_partition_tests = stats.exact_partition_tests + 1
                        parts = Geometry.hpartition_triangle_by_triangle(piece.simplex, parent.simplex)
                    end

                    local expected_rows = (piece_type == "triangle") and 3 or 2
                    local normalized = normalize_partition_parts(parts, expected_rows)
                    if normalized ~= nil then
                        stats.successful_splits = stats.successful_splits + 1
                        stats.pieces_created = stats.pieces_created + #normalized
                        for _, part in ipairs(normalized) do
                            local new_piece = {
                                simplex = part,
                                type = piece_type,
                                bbox2 = part:get_bbox2(),
                            }
                            local part_meta = meta
                            if piece_type == "triangle" then
                                part_meta = copy_simplex_metadata_for_part(piece, part)
                            end
                            for k, v in pairs(part_meta) do new_piece[k] = v end
                            partition_recursive(new_piece, depth + 1, pi)
                        end
                        split_occurred = true
                        break
                    end
                end
            end
        end

        if not split_occurred then
            table.insert(result, piece)
        end
    end

    for _, simplex in ipairs(simplices) do
        partition_recursive(simplex)
    end

    stats.output_pieces = #result
    return result, stats
end


--- Compute an affine Frenet frame for a regular 3D parametric curve.
--- Central finite differences approximate the first and second derivatives at
--- the requested parameter.  The returned Matrix has rows T, N, B, P, where
--- T, N, and B are direction vectors (homogeneous coordinate 0) and P is the
--- curve point (homogeneous coordinate 1).  At zero curvature, choose an
--- arbitrary deterministic normal orthogonal to T so straight portions still
--- yield a usable affine frame.
--- @param curve function number -> Vector homogeneous 3D parametric map
--- @param parameter number parameter value at which to evaluate the frame
--- @param step number positive finite-difference step
--- @return Matrix affine Frenet frame with rows T, N, B, P
function Geometry.frenet(curve, parameter, step)
    local function finite_number(value)
        return type(value) == "number"
            and value == value
            and value ~= math.huge
            and value ~= -math.huge
    end

    local function valid_point(point)
        if getmetatable(point) ~= Vector or #point ~= 4 then return false end
        for i = 1, 4 do
            if not finite_number(point[i]) then return false end
        end
        return true
    end

    local function fallback_frame(point, reason)
        Recovery.log_once("frenet:" .. tostring(reason), "Frenet frame recovered", reason)
        local P = valid_point(point) and point:copy() or Vector:new{0, 0, 0, 1}
        local T = Vector:new{1, 0, 0, 0}
        local N = Vector:new{0, 1, 0, 0}
        local B = Vector:new{0, 0, 1, 0}
        P[4] = 1
        return Matrix:new{T, N, B, P}
    end

    if type(curve) ~= "function" then
        return fallback_frame(nil, "curve was not callable; using a canonical frame")
    end
    if not finite_number(parameter) then
        parameter = 0
    end
    if not (finite_number(step) and step > 0) then
        step = 1e-5
    end

    local function point_at(t)
        local ok, p = pcall(curve, t)
        if ok and valid_point(p) then return p end
        return nil, ok and "curve returned an invalid point" or p
    end

    local p0, err0 = point_at(parameter)
    if not p0 then return fallback_frame(nil, err0) end
    local pm, errm = point_at(parameter - step)
    local pp, errp = point_at(parameter + step)
    if not pm then return fallback_frame(p0, errm) end
    if not pp then return fallback_frame(p0, errp) end

    local first = pp:_hsub(pm):_hscale(1 / (2 * step))
    local second = pp:_hsub(p0:_hscale(2)):_hadd(pm):_hscale(1 / (step * step))

    local first_norm = first:hnorm()
    if not (finite_number(first_norm) and first_norm > 0) then
        return fallback_frame(p0, "curve tangent vanished or became non-finite")
    end

    local T = first:hnormalize()
    local second_norm = second:hnorm()
    local binormal_raw = first:_hcross(second)
    local binormal_norm = binormal_raw:hnorm()
    local curvature_scale = first_norm * second_norm
    local N, B
    if finite_number(second_norm)
        and finite_number(binormal_norm)
        and curvature_scale > 0
        and binormal_norm > GEOMETRY_REL_EPS * curvature_scale
    then
        B = binormal_raw:hnormalize()
        N = B:_hcross(T):hnormalize()
    else
        N = T:horthogonal_vector():hnormalize()
        B = T:_hcross(N):hnormalize()
        N = B:_hcross(T):hnormalize()
    end
    local P = p0:copy()

    T[4], N[4], B[4], P[4] = 0, 0, 0, 1
    return Matrix:new{T, N, B, P}
end


return Geometry
