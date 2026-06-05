--- Scene class
--- @class Scene
local Scene = {}
Scene.__index = Scene

local Vector = nil
local Matrix = nil
local Geometry = nil

--- Vector and Matrix and Geometry injection
--- @param vclass Vector the class is its own metatable
function Scene:_set_classes(vclass, matclass, gclass)
    Vector = vclass
    Matrix = matclass
    Geometry = gclass
end


--- MAIN TABLE
local lua_tikz3dtools = {}

local function make_readonly_table(value, label)
    return setmetatable(value, {
        __newindex = function(_, key, _)
            error(("sandbox value '%s' is read-only; cannot assign '%s'")
                :format(label, tostring(key)), 2)
        end,
        __metatable = false,
    })
end

local function make_readonly_proxy(source, label)
    return setmetatable({}, {
        __index = source,
        __newindex = function(_, key, _)
            error(("sandbox value '%s' is read-only; cannot assign '%s'")
                :format(label, tostring(key)), 2)
        end,
        __metatable = false,
    })
end

--- Build the base environment used by all sandbox evaluations.
--- User-defined objects are resolved separately so expressions cannot mutate
--- shared global state or shadow built-in names.
local blocked_globals = {
    debug = true,
    dofile = true,
    load = true,
    loadfile = true,
    package = true,
    require = true,
}

local proxied_tables = {
    coroutine = true,
    io = true,
    math = true,
    os = true,
    string = true,
    table = true,
    utf8 = true,
}

local function build_base_env()
    local env = {}

    for key, value in pairs(_G) do
        if not blocked_globals[key] then
            if proxied_tables[key] and type(value) == "table" then
                env[key] = make_readonly_proxy(value, key)
            else
                env[key] = value
            end
        end
    end

    env.table = env.table or make_readonly_proxy(table, "table")
    env.math = env.math or make_readonly_proxy(math, "math")

    -- Expose math entries directly so callers can use sin/cos/etc. without math. prefix
    for k, v in pairs(math) do
        if env[k] == nil then
            env[k] = v
        end
    end

    return env
end

local function source_preview(str)
    local preview = tostring(str or "")
        :gsub("%s+", " ")
        :gsub("^%s+", "")

    if #preview > 160 then
        preview = preview:sub(1, 157) .. "..."
    end

    return preview
end

local function format_eval_error(kind, label, source, err)
    return ("%s in %s: %s\nSource: %s")
        :format(kind, label or "expression", tostring(err), source_preview(source))
end

local function make_eval_env(bindings)
    bindings = bindings or {}

    return setmetatable({}, {
        __index = function(_, key)
            local value = bindings[key]
            if value ~= nil then
                return value
            end

            value = lua_tikz3dtools.objects[key]
            if value ~= nil then
                return value
            end

            return lua_tikz3dtools.base_env[key]
        end,
        __newindex = function(_, key, _)
            error(("sandbox is read-only; cannot assign global '%s'")
                :format(tostring(key)), 2)
        end,
        __metatable = false,
    })
end

local function evaluate_chunk(source, label, bindings)
    local chunk, syntax_err = load(source, label or "expression", "t", make_eval_env(bindings))

    if not chunk then
        error(format_eval_error("Lua syntax error", label, source, syntax_err), 0)
    end

    local ok, result = pcall(chunk)
    if not ok then
        error(format_eval_error("Lua evaluation error", label, source, result), 0)
    end

    return result
end

local function wrap_user_function(fn, label, source)
    return function(...)
        local ok, result = pcall(fn, ...)
        if not ok then
            error(format_eval_error("Lua function error", label, source, result), 0)
        end
        return result
    end
end

local statement_keywords = {
    "return",
    "local",
    "if",
    "for",
    "while",
    "repeat",
    "do",
}

local function starts_with_statement(trimmed)
    for _, keyword in ipairs(statement_keywords) do
        if trimmed:match("^" .. keyword .. "%f[%W]") then
            return true
        end
    end

    return false
end

-- ================================================================
-- TeX command registration helper
-- https://tex.stackexchange.com/a/747040
-- ================================================================

local function register_tex_cmd(name, func, args, protected)
    name = "__lua_tikztdtools_" .. name .. ":" .. ("n"):rep(#args)
    local scanners = {}
    for _, arg in ipairs(args) do
        scanners[#scanners+1] = token['scan_' .. arg]
    end
    local scanning_func = function()
        local values = {}
        for _, scanner in ipairs(scanners) do
            values[#values+1] = scanner()
        end
        func(table.unpack(values))
    end
    local index = luatexbase.new_luafunction(name)
    lua.get_functions_table()[index] = scanning_func
    if protected then
        token.set_lua(name, index, "protected")
    else
        token.set_lua(name, index)
    end
end

lua_tikz3dtools.simplices = {}
lua_tikz3dtools.lights = {}
lua_tikz3dtools.objects = {}
lua_tikz3dtools.base_env = build_base_env()

local function refresh_base_env()
    lua_tikz3dtools.base_env.Vector = make_readonly_proxy(Vector, "Vector")
    lua_tikz3dtools.base_env.Matrix = make_readonly_proxy(Matrix, "Matrix")
    lua_tikz3dtools.base_env.tau = 2 * math.pi
end

--- Late-init: called after Vector/Matrix are set to update the env table
function Scene._init_math_env()
    refresh_base_env()
end

-- ================================================================
-- Expression evaluators (sandboxed)
-- ================================================================

local function single_string_expression(str, label, bindings)
    return evaluate_chunk(("return %s"):format(str), label, bindings)
end

local function body_expression(str, label, bindings)
    return evaluate_chunk(str, label, bindings)
end

local function object_expression(str, label, bindings)
    local trimmed = str:match("^%s*(.-)%s*$") or ""
    if starts_with_statement(trimmed) then
        return body_expression(str, label, bindings)
    end
    return single_string_expression(str, label, bindings)
end

local function options_string_expression(str, label, bindings)
    label = label or "options expression"

    if type(str) ~= "string" or str == "" then
        return ""
    end

    local trimmed = str:match("^%s*(.-)%s*$") or ""
    if trimmed == "" then
        return ""
    end

    if starts_with_statement(trimmed) then
        local value = body_expression(str, label, bindings)
        assert(type(value) == "string", label .. " must return a string")
        return value
    end

    local ok, value_or_err = pcall(object_expression, str, label, bindings)
    if ok then
        assert(type(value_or_err) == "string", label .. " must evaluate to a string")
        return value_or_err
    end

    if tostring(value_or_err):match("^Lua syntax error") then
        return str
    end

    error(value_or_err, 0)
end

local function single_string_function(str, label, bindings)
    local fn = evaluate_chunk(("return function(u) %s end"):format(str), label, bindings)
    return wrap_user_function(fn, label, str)
end

local function double_string_function(str, label, bindings)
    local fn = evaluate_chunk(("return function(u,v) %s end"):format(str), label, bindings)
    return wrap_user_function(fn, label, str)
end

local function triple_string_function(str, label, bindings)
    local fn = evaluate_chunk(("return function(u,v,w) %s end"):format(str), label, bindings)
    return wrap_user_function(fn, label, str)
end

local function is_finite_number(value)
    return type(value) == "number"
        and value == value
        and value ~= math.huge
        and value ~= -math.huge
end

local function is_finite_numeric_matrix(value)
    if getmetatable(value) ~= Matrix then
        return false
    end

    for i = 1, #value do
        for j = 1, #value[i] do
            if not is_finite_number(value[i][j]) then
                return false
            end
        end
    end

    return true
end

local function is_finite_simplex(simplex)
    local mt = getmetatable(simplex)

    if mt == Vector then
        for i = 1, #simplex do
            if not is_finite_number(simplex[i]) then
                return false
            end
        end
        return true
    end

    if mt == Matrix then
        for i = 1, #simplex do
            for j = 1, #simplex[i] do
                if not is_finite_number(simplex[i][j]) then
                    return false
                end
            end
        end
        return true
    end

    return false
end

local function project_point(v, transformation, label)
    if not v then
        return nil
    end

    if not is_finite_numeric_matrix(transformation) then
        return nil
    end

    local ok, projected = pcall(function()
        return v:_multiply(transformation)
    end)

    if not ok then
        error(("Projection failed for %s: %s")
            :format(label or "point", tostring(projected)), 0)
    end

    if is_finite_simplex(projected) then
        return projected
    end

    return nil
end

local function project_simplex(simplex, transformation, label)
    if not simplex then
        return nil
    end

    if not is_finite_numeric_matrix(transformation) then
        return nil
    end

    local ok, projected = pcall(function()
        return simplex:_multiply(transformation):reciprocate_by_homogeneous()
    end)

    if not ok then
        error(("Projection failed for %s: %s")
            :format(label or "simplex", tostring(projected)), 0)
    end

    if is_finite_simplex(projected) then
        return projected
    end

    return nil
end

local function push_simplex(entry)
    if entry.simplex and is_finite_simplex(entry.simplex) then
        table.insert(lua_tikz3dtools.simplices, entry)
        return true
    end

    return false
end

local function is_nonempty_string(value)
    return type(value) == "string" and value ~= ""
end

local DEFAULT_ARROW_SCALE = 0.5

local function resolve_arrow_scale(scale_src, fallback, label)
    local scale_label = label or "arrow scale"

    if scale_src == nil or scale_src == "" then
        return fallback or DEFAULT_ARROW_SCALE
    end

    local scale = object_expression(scale_src, scale_label)
    assert(is_finite_number(scale) and scale > 0,
        scale_label .. " must evaluate to a positive number")
    return scale
end

local function uv_curve_point(value)
    if getmetatable(value) ~= Vector or #value < 2 then
        return nil
    end
    if not is_finite_number(value[1]) or not is_finite_number(value[2]) then
        return nil
    end
    return Vector:_new{value[1], value[2], 1}
end

local function uv_curve_point_value(value)
    if getmetatable(value) == Vector then
        return uv_curve_point(value)
    end
    if type(value) == "table" then
        return uv_curve_point(Vector:_new(value))
    end
    return nil
end

local function append_uv_curve_segment(uv_segments, start_point, stop_point, drawoptions)
    if start_point and stop_point and start_point:_hdistance(stop_point) > 1e-12 then
        table.insert(uv_segments, {
            simplex = Matrix:_new{start_point, stop_point},
            drawoptions = drawoptions,
        })
    end
end

local function append_uv_arrow_segments(uv_segments, tip_point, tail_point, drawoptions, scale)
    if not is_nonempty_string(drawoptions) then
        return
    end

    local direction = tip_point:_hsub(tail_point)
    local length = direction:hnorm()
    if length <= 1e-12 then
        return
    end

    local tip_scale = math.min(scale or DEFAULT_ARROW_SCALE, length)
    if tip_scale <= 1e-12 then
        return
    end

    local U = direction:hnormalize()
    local V = U:_multiply(Matrix:_new{
        Vector:_new{0, 1, 0},
        Vector:_new{-1, 0, 0},
        Vector:_new{0, 0, 1}
    }):hnormalize()
    local base_point = tip_point:_hsub(U:_hscale(tip_scale))

    append_uv_curve_segment(
        uv_segments,
        base_point:_hadd(V:_hscale(tip_scale)),
        tip_point,
        drawoptions
    )
    append_uv_curve_segment(
        uv_segments,
        base_point:_hsub(V:_hscale(tip_scale)),
        tip_point,
        drawoptions
    )
end

local function explicit_uv_curve_segments(str, label)
    label = label or "surface curve"

    local segments = body_expression(str, label)
    local uv_segments = {}

    if type(segments) ~= "table" then
        return nil
    end

    for _, segment in ipairs(segments) do
        if type(segment) == "table" or getmetatable(segment) == Matrix then
            local P = uv_curve_point_value(segment.start or segment[1])
            local Q = uv_curve_point_value(segment.stop or segment[2])

            if P and Q and P:_hdistance(Q) > 1e-12 then
                local arrowscale = segment.arrowscale
                if arrowscale == nil then
                    arrowscale = DEFAULT_ARROW_SCALE
                end
                if arrowscale ~= nil then
                    assert(is_finite_number(arrowscale) and arrowscale > 0,
                        label .. ".arrowscale must be a positive number")
                end

                append_uv_curve_segment(uv_segments, P, Q, segment.drawoptions)

                if is_nonempty_string(segment.arrowtail) then
                    append_uv_arrow_segments(uv_segments, P, Q, segment.arrowtail, arrowscale)
                end

                if is_nonempty_string(segment.arrowtip) then
                    append_uv_arrow_segments(uv_segments, Q, P, segment.arrowtip, arrowscale)
                end
            end
        end
    end

    if #uv_segments == 0 then
        return nil
    end
    return uv_segments
end

local function embedded_segments_in_triangle(uv_segments, uv_triangle)
    if uv_segments == nil then
        return nil
    end

    local embedded_segments = {}
    for _, segment in ipairs(uv_segments) do
        local orig_start = Vector:_new(segment.simplex[1])
        local orig_stop  = Vector:_new(segment.simplex[2])
        local clipped = Geometry.hclip_line_segment_to_triangle(segment.simplex, uv_triangle)
        if clipped ~= nil then
            local start_bary = Geometry.hpoint_triangle_barycentric(Vector:_new(clipped[1]), uv_triangle)
            local stop_bary = Geometry.hpoint_triangle_barycentric(Vector:_new(clipped[2]), uv_triangle)
            if start_bary ~= nil and stop_bary ~= nil then
                table.insert(embedded_segments, {
                    start = start_bary,
                    stop = stop_bary,
                    drawoptions = segment.drawoptions,
                    start_cut = Vector:_new(clipped[1]):_hdistance(orig_start) > 1e-9,
                    stop_cut  = Vector:_new(clipped[2]):_hdistance(orig_stop)  > 1e-9,
                })
            end
        end
    end

    if #embedded_segments == 0 then
        return nil
    end

    return embedded_segments
end

local function triangle_shading_normal(simplex)
    local A = Vector:_new(simplex[1])
    local B = Vector:_new(simplex[2])
    local C = Vector:_new(simplex[3])
    local normal = (B:_hsub(A)):_hcross(C:hsub(A))

    if normal:hnorm() <= 1e-12 then
        return nil
    end

    return normal:hnormalize()
end

local function light_facing_normal(normal, light_dir)
    if normal:_hinner(light_dir) < 0 then
        return normal:_hscale(-1)
    end
    return normal
end

local function render_embedded_segments(simplex)
    if simplex.embedded_segments == nil then
        return
    end

    local T = simplex.simplex
    local ax, ay = T[1][1], T[1][2]
    local bx, by = T[2][1], T[2][2]
    local cx, cy = T[3][1], T[3][2]

    local max_edge = math.sqrt(math.max(
        (ax-bx)^2 + (ay-by)^2,
        (bx-cx)^2 + (by-cy)^2,
        (cx-ax)^2 + (cy-ay)^2
    ))

    for _, segment in ipairs(simplex.embedded_segments) do
        local start_point = Geometry.hpoint_from_triangle_barycentric(
            T, Vector:_new(segment.start)
        )
        local stop_point = Geometry.hpoint_from_triangle_barycentric(
            T, Vector:_new(segment.stop)
        )

        if start_point:_hdistance(stop_point) > 1e-12 then
            local sx, sy = start_point[1], start_point[2]
            local ex, ey = stop_point[1], stop_point[2]
            local s_cut = segment.start_cut == true
            local e_cut = segment.stop_cut  == true

            if (s_cut or e_cut) and max_edge > 1e-12 then
                local dx, dy = ex - sx, ey - sy
                local seg_len = math.sqrt(dx*dx + dy*dy)
                if seg_len > 1e-12 then
                    local ux, uy = dx / seg_len, dy / seg_len
                    local draw_sx = s_cut and (sx - ux * max_edge) or sx
                    local draw_sy = s_cut and (sy - uy * max_edge) or sy
                    local draw_ex = e_cut and (ex + ux * max_edge) or ex
                    local draw_ey = e_cut and (ey + uy * max_edge) or ey
                    tex.sprint(
                        ("\\begin{scope}\\clip (%f,%f)--(%f,%f)--(%f,%f)--cycle;\\path[%s] (%f,%f)--(%f,%f);\\end{scope}")
                        :format(
                            ax, ay, bx, by, cx, cy,
                            segment.drawoptions or "",
                            draw_sx, draw_sy, draw_ex, draw_ey
                        )
                    )
                end
            else
                tex.sprint(
                    ("\\path[%s] (%f,%f) -- (%f,%f);")
                    :format(
                        segment.drawoptions or "",
                        sx, sy, ex, ey
                    )
                )
            end
        end
    end
end

local function param_triplet(value)
    assert(value and getmetatable(value) == Vector,
        "axis params must return a Vector")
    assert(value[1] ~= nil and value[2] ~= nil and value[3] ~= nil,
        "axis params must contain start, stop, and samples")

    return value[1], value[2], value[3]
end

local function resolve_axis_params(params_src)
    return param_triplet(body_expression(params_src))
end

local append_solid

local function append_surface(hash)
    local ustart, ustop, usamples = resolve_axis_params(hash.uparams)
    local vstart, vstop, vsamples = resolve_axis_params(hash.vparams)
    local transformation = object_expression(hash.transformation)
    local f              = double_string_function(hash.v)
    local filloptions    = options_string_expression(hash.filloptions)
    local filter         = hash.filter
    local uv_curve_segments

    assert(usamples and usamples >= 2, "usamples must be >= 2, got: " .. tostring(usamples))
    assert(vsamples and vsamples >= 2, "vsamples must be >= 2, got: " .. tostring(vsamples))

    local ustep = (ustop - ustart) / (usamples - 1)
    local vstep = (vstop - vstart) / (vsamples - 1)

    local function parametric_surface(u, v)
        return f(u, v)
    end

    if is_nonempty_string(hash.curve) then
        uv_curve_segments = explicit_uv_curve_segments(hash.curve)
    end

    for i = 0, usamples - 2 do
        local u = ustart + i * ustep
        for j = 0, vsamples - 2 do
            local v = vstart + j * vstep
            local A = parametric_surface(u, v)
            local B = parametric_surface(u + ustep, v)
            local C = parametric_surface(u + ustep, v + vstep)
            local D = parametric_surface(u, v + vstep)
            if A and B and C and D then
                local uvA = Vector:_new{u, v, 1}
                local uvB = Vector:_new{u + ustep, v, 1}
                local uvC = Vector:_new{u + ustep, v + vstep, 1}
                local uvD = Vector:_new{u, v + vstep, 1}
                if not (
                    Geometry.hpoint_point_intersecting(A, B)
                    or Geometry.hpoint_point_intersecting(B, C)
                    or Geometry.hpoint_point_intersecting(A, C)
                ) then
                    local simplex1 = project_simplex(
                        Matrix:_new{A, B, C},
                        transformation
                    )
                    if simplex1 then
                        push_simplex({
                            simplex           = simplex1,
                            filloptions       = filloptions,
                            type              = "triangle",
                            filter            = filter,
                            shading_normal    = triangle_shading_normal(simplex1),
                            embedded_segments = embedded_segments_in_triangle(
                                uv_curve_segments,
                                Matrix:_new{uvA, uvB, uvC}
                            )
                        })
                    end
                end
                if not (
                    Geometry.hpoint_point_intersecting(A, D)
                    or Geometry.hpoint_point_intersecting(D, C)
                    or Geometry.hpoint_point_intersecting(A, C)
                ) then
                    local simplex2 = project_simplex(
                        Matrix:_new{A, D, C},
                        transformation
                    )
                    if simplex2 then
                        push_simplex({
                            simplex           = simplex2,
                            filloptions       = filloptions,
                            type              = "triangle",
                            filter            = filter,
                            shading_normal    = triangle_shading_normal(simplex2),
                            embedded_segments = embedded_segments_in_triangle(
                                uv_curve_segments,
                                Matrix:_new{uvA, uvD, uvC}
                            )
                        })
                    end
                end
            end
        end
    end
end

local function arrow_basis(tip_point, tail_point)
    local direction = tip_point:_hsub(tail_point)
    if direction:hnorm() <= 1e-12 then
        return nil
    end

    local U = direction:hnormalize()
    local V = U:horthogonal_vector():hnormalize()
    local W = U:_hcross(V):hnormalize()
    

    return U, V, W
end

local function append_projected_arrow_surface(tip_point, tail_point, filloptions, scale, filter, kind)
    if not is_nonempty_string(filloptions) then
        return
    end

    if not (tip_point and tail_point) or tip_point:_hdistance(tail_point) <= 1e-12 then
        return
    end

    local U, V, W = arrow_basis(tip_point, tail_point)
    if not (U and V and W) then
        return
    end

    local transformation_entries = {
        scale * W[1], scale * W[2], scale * W[3],
        scale * V[1], scale * V[2], scale * V[3],
        scale * U[1], scale * U[2], scale * U[3],
        tip_point[1], tip_point[2], tip_point[3],
    }

    for _, entry in ipairs(transformation_entries) do
        if not is_finite_number(entry) then
            return
        end
    end

    local transformation_src = ([[
        return Matrix:new{
            Vector:new{%f,%f,%f,0},
            Vector:new{%f,%f,%f,0},
            Vector:new{%f,%f,%f,0},
            Vector:new{%f,%f,%f,1}
        }
    ]]):format(table.unpack(transformation_entries))

    if kind == "tail" then
        append_solid{
            uparams = "return Vector:new{0, tau, 10}",
            vparams = "return Vector:new{0, 0.25, 2}",
            wparams = "return Vector:new{-0.5, 0.5, 2}",
            v = "return Vector:_new{v*cos(u), v*sin(u), w, 1}",
            filloptions = filloptions,
            transformation = transformation_src,
            filter = filter
        }
        return
    end

    -- Render tip as a solid cone so the underside/base is visible.
    append_solid{
        uparams = "return Vector:new{0, tau, 10}",
        vparams = "return Vector:new{0, 0.25, 2}",
        wparams = "return Vector:new{0, 1, 2}",
        v = "return Vector:_new{v*w*math.cos(u), v*w*math.sin(u), -w, 1}",
        filloptions = filloptions,
        transformation = transformation_src,
        filter = filter
    }
end

local function append_point(hash)
    local v              = body_expression(hash.v)
    local transformation = object_expression(hash.transformation)
    local filloptions    = options_string_expression(hash.filloptions)
    local filter         = hash.filter
    if v then
        local the_simplex = project_point(v, transformation)
        if the_simplex then
            push_simplex({
                simplex     = the_simplex,
                filloptions = filloptions,
                type        = "point",
                filter      = filter
            })
        end
    end
end

local function append_triangle(hash)
    local transformation = object_expression(hash.transformation)
    local filter         = hash.filter
    local filloptions    = options_string_expression(hash.filloptions)
    assert(hash.m and hash.m ~= "", "appendtriangle.m must return a 3-row Matrix")

    local the_simplex = object_expression(hash.m)
    assert(getmetatable(the_simplex) == Matrix, "appendtriangle.m must return a Matrix")
    assert(#the_simplex == 3, "appendtriangle.m must return a 3-row Matrix")

    local A = Vector:_new(the_simplex[1])
    local B = Vector:_new(the_simplex[2])
    local C = Vector:_new(the_simplex[3])

    if not (
        Geometry.hpoint_point_intersecting(A, B)
        or Geometry.hpoint_point_intersecting(B, C)
        or Geometry.hpoint_point_intersecting(C, A)
    ) then
        local projected = project_simplex(the_simplex, transformation)
        if projected then
            push_simplex({
                simplex     = projected,
                filloptions = filloptions,
                type        = "triangle",
                filter      = filter,
                shading_normal = triangle_shading_normal(projected)
            })
        end
    end
end

local function append_label(hash)
    local v              = body_expression(hash.v)
    local filter         = hash.filter
    local text           = hash.text
    local transformation = object_expression(hash.transformation)
    if v then
        local the_simplex = project_point(v, transformation)
        if the_simplex then
            push_simplex({
                simplex     = the_simplex,
                text        = text,
                type        = "label",
                filter      = filter
            })
        end
    end
end

local function append_light(hash)
    local v = body_expression(hash.v)
    if v and getmetatable(v) == Vector then
        table.insert(lua_tikz3dtools.lights, v)
    else
        error("Invalid light vector: " .. tostring(v))
    end
end


local function append_curve(hash)
    local ustart, ustop, usamples = resolve_axis_params(hash.uparams)
    local transformation = object_expression(hash.transformation)
    local f              = single_string_function(hash.v)
    local filter         = hash.filter
    local drawoptions    = options_string_expression(hash.drawoptions)
    local arrowoptions   = options_string_expression(hash.arrowtip)
    local tailoptions    = options_string_expression(hash.arrowtail)
    local arrowscale     = resolve_arrow_scale(hash.arrowscale, DEFAULT_ARROW_SCALE)

    assert(usamples and usamples >= 2, "usamples must be >= 2, got: " .. tostring(usamples))

    local ustep = (ustop - ustart) / (usamples - 1)

    local function parametric_curve(u)
        return f(u)
    end

    for i = 0, usamples - 2 do
        local u = ustart + i * ustep
        local A = parametric_curve(u)
        local B = parametric_curve(u + ustep)
        if A and B then
            local simplex = project_simplex(
                Matrix:_new{A, B},
                transformation
            )
            if simplex then
                push_simplex({
                    simplex      = simplex,
                    drawoptions  = drawoptions,
                    type         = "line segment",
                    filter       = filter
                })
            end
            if i == 0 and is_nonempty_string(tailoptions) then
                local P = project_point(parametric_curve(ustart), transformation)
                local Q = project_point(parametric_curve(ustart + ustep), transformation)
                append_projected_arrow_surface(P, Q, tailoptions, arrowscale, filter, "tail")
            end
            if i == usamples - 2 and is_nonempty_string(arrowoptions) then
                local P = project_point(parametric_curve(ustop), transformation)
                local Q = project_point(parametric_curve(ustop - ustep), transformation)
                append_projected_arrow_surface(P, Q, arrowoptions, arrowscale, filter, "tip")
            end
        end
    end
end

append_solid = function(hash)
    local ustart, ustop, usamples = resolve_axis_params(hash.uparams)
    local vstart, vstop, vsamples = resolve_axis_params(hash.vparams)
    local wstart, wstop, wsamples = resolve_axis_params(hash.wparams)
    local filloptions    = options_string_expression(hash.filloptions)
    local filter = hash.filter
    local transformation = object_expression(hash.transformation)
    local f = triple_string_function(hash.v)

    assert(usamples and usamples >= 2, "usamples must be >= 2, got: " .. tostring(usamples))
    assert(vsamples and vsamples >= 2, "vsamples must be >= 2, got: " .. tostring(vsamples))
    assert(wsamples and wsamples >= 2, "wsamples must be >= 2, got: " .. tostring(wsamples))

    local function parametric_solid(u, v, w)
        return f(u, v, w)
    end

    local ustep = (ustop - ustart) / (usamples - 1)
    local vstep = (vstop - vstart) / (vsamples - 1)
    local wstep = (wstop - wstart) / (wsamples - 1)

    local function tessellate_face(fixed_var, fixed_val, s1_start, s1_step, s1_count, s2_start, s2_step, s2_count)
        for i = 0, s1_count - 2 do
            local s1 = s1_start + i * s1_step
            for j = 0, s2_count - 2 do
                local s2 = s2_start + j * s2_step
                local A, B, C, D
                if fixed_var == "u" then
                    A = parametric_solid(fixed_val, s1, s2)
                    B = parametric_solid(fixed_val, s1 + s1_step, s2)
                    C = parametric_solid(fixed_val, s1 + s1_step, s2 + s2_step)
                    D = parametric_solid(fixed_val, s1, s2 + s2_step)
                elseif fixed_var == "v" then
                    A = parametric_solid(s1, fixed_val, s2)
                    B = parametric_solid(s1 + s1_step, fixed_val, s2)
                    C = parametric_solid(s1 + s1_step, fixed_val, s2 + s2_step)
                    D = parametric_solid(s1, fixed_val, s2 + s2_step)
                elseif fixed_var == "w" then
                    A = parametric_solid(s1, s2, fixed_val)
                    B = parametric_solid(s1 + s1_step, s2, fixed_val)
                    C = parametric_solid(s1 + s1_step, s2 + s2_step, fixed_val)
                    D = parametric_solid(s1, s2 + s2_step, fixed_val)
                end
                if A and B and D then
                    local simplex = project_simplex(
                        Matrix:_new{A, B, D},
                        transformation
                    )
                    if simplex then
                        push_simplex({
                            simplex     = simplex,
                            filloptions = filloptions,
                            type        = "triangle",
                            filter      = filter,
                            shading_normal = triangle_shading_normal(simplex)
                        })
                    end
                end
                if B and C and D then
                    local simplex = project_simplex(
                        Matrix:_new{B, C, D},
                        transformation
                    )
                    if simplex then
                        push_simplex({
                            simplex     = simplex,
                            filloptions = filloptions,
                            type        = "triangle",
                            filter      = filter,
                            shading_normal = triangle_shading_normal(simplex)
                        })
                    end
                end
            end
        end
    end

    tessellate_face("u", ustart, vstart, vstep, vsamples, wstart, wstep, wsamples)
    tessellate_face("u", ustop,  vstart, vstep, vsamples, wstart, wstep, wsamples)
    tessellate_face("v", vstart, ustart, ustep, usamples, wstart, wstep, wsamples)
    tessellate_face("v", vstop,  ustart, ustep, usamples, wstart, wstep, wsamples)
    tessellate_face("w", wstart, ustart, ustep, usamples, vstart, vstep, vsamples)
    tessellate_face("w", wstop,  ustart, ustep, usamples, vstart, vstep, vsamples)
end

local function apply_filters(simplices)
    local new_simplices = {}

    for _, simplex in ipairs(simplices) do
        local bindings = {}

        if simplex.type == "point" then
            bindings.A = Vector:_new(simplex.simplex)
        elseif simplex.type == "line segment" then
            bindings.A = Vector:_new(simplex.simplex[1])
            bindings.B = Vector:_new(simplex.simplex[2])
        elseif simplex.type == "triangle" then
            bindings.A = Vector:_new(simplex.simplex[1])
            bindings.B = Vector:_new(simplex.simplex[2])
            bindings.C = Vector:_new(simplex.simplex[3])
        elseif simplex.type == "label" then
            bindings.A = Vector:_new(simplex.simplex)
        end

        local filter_body = simplex.filter or "return true"
        local filter_label = ("filter[%s]"):format(simplex.type)
        local filter_fn = evaluate_chunk(
            ("return function()\n%s\nend"):format(filter_body),
            filter_label,
            bindings
        )

        if wrap_user_function(filter_fn, filter_label, filter_body)() then
            table.insert(new_simplices, simplex)
        end
    end

    return new_simplices
end

local function display_simplices()
    print("\nTime:" .. os.date("%X") .. " Displaying " .. #lua_tikz3dtools.simplices .. " simplices.")

    -- Pre-compute bbox2 for all simplices
    for _, s in ipairs(lua_tikz3dtools.simplices) do
        if s.type ~= "point" and s.type ~= "label" then
            s.bbox2 = s.simplex:get_bbox2()
        end
    end

    lua_tikz3dtools.simplices = Geometry.partition_simplices_by_parents(
        lua_tikz3dtools.simplices,
        lua_tikz3dtools.simplices
    )
    print("Time:" .. os.date("%X") .. " After partitioning, " .. #lua_tikz3dtools.simplices .. " simplices remain.")

    lua_tikz3dtools.simplices = apply_filters(lua_tikz3dtools.simplices)
    print("Time:" .. os.date("%X") .. " After filtering, " .. #lua_tikz3dtools.simplices .. " simplices remain.")

    -- Re-compute bbox2 after filtering (some simplices removed)
    for _, s in ipairs(lua_tikz3dtools.simplices) do
        if s.type ~= "point" and s.type ~= "label" and not s.bbox2 then
            s.bbox2 = s.simplex:get_bbox2()
        end
    end

    lua_tikz3dtools.simplices = Geometry.scc(lua_tikz3dtools.simplices)
    print("Time:" .. os.date("%X") .. " Occlusion sortinc complete.")

    local labels = {}
    for _, simplex in ipairs(lua_tikz3dtools.simplices) do
        if simplex.type == "point" then
            tex.sprint(
                ("\\path[%s] (%f,%f) circle[radius = 0.06];")
                :format(simplex.filloptions, simplex.simplex[1], simplex.simplex[2])
            )
        elseif simplex.type == "line segment" then
            tex.sprint(
                ("\\path[%s] (%f,%f) -- (%f,%f);")
                :format(
                    simplex.drawoptions,
                    simplex.simplex[1][1], simplex.simplex[1][2],
                    simplex.simplex[2][1], simplex.simplex[2][2]
                )
            )
        elseif simplex.type == "triangle" then
            local num_lights = #lua_tikz3dtools.lights
            if num_lights > 0 then
                local normal = simplex.shading_normal or triangle_shading_normal(simplex.simplex)

                local total_intensity = 0
                if normal then
                    for _, light in ipairs(lua_tikz3dtools.lights) do
                        local light_dir = light:hnormalize()
                        local facing_normal = light_facing_normal(normal, light_dir)
                        local cos_theta = facing_normal:hinner(light_dir)
                        if cos_theta > 1 then cos_theta = 1 end
                        -- Linear falloff: 0° → 1.0, 90° → 0.0
                        local theta = math.deg(math.acos(cos_theta))
                        total_intensity = total_intensity + (1 - theta / 90)
                    end
                end

                local avg_intensity = math.floor((total_intensity / num_lights) * 100 + 0.01)
                tex.sprint(("\\colorlet{ltdtbrightness}{white!%f!black}"):format(avg_intensity))
            else
                tex.sprint(("\\colorlet{ltdtbrightness}{white!%f!black}"):format(0))
            end

            tex.sprint(
                ("\\path[%s] (%f,%f) -- (%f,%f) -- (%f,%f) -- cycle;")
                :format(
                    simplex.filloptions,
                    simplex.simplex[1][1], simplex.simplex[1][2],
                    simplex.simplex[2][1], simplex.simplex[2][2],
                    simplex.simplex[3][1], simplex.simplex[3][2]
                )
            )
            render_embedded_segments(simplex)
        elseif simplex.type == "label" then
            table.insert(labels, simplex)
        end
    end

    for _, simplex in ipairs(labels) do
        tex.sprint(
            ("\\node at (%f,%f) {%s};")
            :format(simplex.simplex[1], simplex.simplex[2], simplex.text)
        )
    end

    lua_tikz3dtools.simplices = {}
    lua_tikz3dtools.lights = {}
end

local function set_object(hash)
    local object = object_expression(hash.object)
    local name = hash.name

    assert(type(name) == "string" and name ~= "", "setobject.name must be a non-empty string")
    assert(lua_tikz3dtools.base_env[name] == nil,
        ("setobject.name '%s' is reserved and cannot be rebound"):format(name))

    lua_tikz3dtools.objects[name] = object
    return object
end

--- Read a TeX macro, returning a fallback if undefined.
local function get_macro_or(name, fallback)
    local val = token.get_macro(name)
    if val == nil or val == "" then return fallback end
    return val
end

function Scene.register_commands()
    register_tex_cmd("appendpoint", function()
        append_point{
            v              = token.get_macro("luatikztdtools@p@p@v"),
            filloptions    = get_macro_or("luatikztdtools@p@p@filloptions", "return \"\""),
            transformation = get_macro_or("luatikztdtools@p@p@transformation", "return Matrix.identity()"),
            filter         = get_macro_or("luatikztdtools@p@p@filter", "return true")
        }
    end, { })

    register_tex_cmd("appendsurface", function()
        append_surface{
            uparams        = get_macro_or("luatikztdtools@p@s@uparams", "return Vector:new{0,1,10}"),
            vparams        = get_macro_or("luatikztdtools@p@s@vparams", "return Vector:new{0,1,10}"),
            v              = token.get_macro("luatikztdtools@p@s@v"),
            curve          = token.get_macro("luatikztdtools@p@s@curve"),
            transformation = get_macro_or("luatikztdtools@p@s@transformation", "return Matrix.identity()"),
            filloptions    = get_macro_or("luatikztdtools@p@s@filloptions", "return \"\""),
            filter         = get_macro_or("luatikztdtools@p@s@filter", "return true"),
        }
    end, { })

    register_tex_cmd("appendtriangle", function()
        append_triangle{
            m              = token.get_macro("luatikztdtools@p@t@m"),
            transformation = get_macro_or("luatikztdtools@p@t@transformation", "return Matrix.identity()"),
            filloptions    = get_macro_or("luatikztdtools@p@t@filloptions", "return \"\""),
            filter         = get_macro_or("luatikztdtools@p@t@filter", "return true"),
        }
    end, { })

    register_tex_cmd("appendlabel", function()
        append_label{
            v              = token.get_macro("luatikztdtools@p@l@v"),
            text           = token.get_macro("luatikztdtools@p@l@text"),
            transformation = get_macro_or("luatikztdtools@p@l@transformation", "return Matrix.identity()"),
            filter         = get_macro_or("luatikztdtools@p@l@filter", "return true")
        }
    end, { })

    register_tex_cmd("appendlight", function()
        append_light{
            v = token.get_macro("luatikztdtools@p@la@v")
        }
    end, { })

    register_tex_cmd("appendcurve", function()
        append_curve{
            uparams        = get_macro_or("luatikztdtools@p@c@uparams", "return Vector:new{0,1,10}"),
            v              = token.get_macro("luatikztdtools@p@c@v"),
            transformation = get_macro_or("luatikztdtools@p@c@transformation", "return Matrix.identity()"),
            drawoptions    = get_macro_or("luatikztdtools@p@c@drawoptions", "return \"\""),
            arrowtip       = get_macro_or("luatikztdtools@p@c@arrowtip", "return \"\""),
            arrowtail      = get_macro_or("luatikztdtools@p@c@arrowtail", "return \"\""),
            arrowscale     = token.get_macro("luatikztdtools@p@c@arrowscale"),
            filter         = get_macro_or("luatikztdtools@p@c@filter", "return true")
        }
    end, { })

    register_tex_cmd("appendsolid", function()
        append_solid{
            uparams        = get_macro_or("luatikztdtools@p@solid@uparams", "return Vector:new{0,1,10}"),
            vparams        = get_macro_or("luatikztdtools@p@solid@vparams", "return Vector:new{0,1,10}"),
            wparams        = get_macro_or("luatikztdtools@p@solid@wparams", "return Vector:new{0,1,10}"),
            v              = token.get_macro("luatikztdtools@p@solid@v"),
            transformation = get_macro_or("luatikztdtools@p@solid@transformation", "return Matrix.identity()"),
            filloptions    = get_macro_or("luatikztdtools@p@solid@filloptions", "return \"\""),
            filter         = get_macro_or("luatikztdtools@p@solid@filter", "return true")
        }
    end, { })

    register_tex_cmd("displaysimplices", function()
        display_simplices()
    end, { })

    register_tex_cmd("setobject", function()
        set_object{
            name   = token.get_macro("luatikztdtools@p@m@name"),
            object = token.get_macro("luatikztdtools@p@m@object"),
        }
    end, { })
end

return Scene
