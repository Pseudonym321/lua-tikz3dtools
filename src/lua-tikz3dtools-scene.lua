--- Scene class
--- @class Scene
local Scene = {}
Scene.__index = Scene

local Vector = nil
local Matrix = nil
local Geometry = nil
local AABBTree = require "lua-tikz3dtools-aabbtree"
local Adaptive = require "lua-tikz3dtools-adaptive"
local VirtualFile = require "lua-tikz3dtools-virtualfile"
local Recovery = require "lua-tikz3dtools-recovery"

--- Vector and Matrix and Geometry injection
--- @param vclass Vector the class is its own metatable
function Scene:_set_classes(vclass, matclass, gclass)
    Vector = vclass
    Matrix = matclass
    Geometry = gclass
    Adaptive:_set_classes(vclass)
end


--- MAIN TABLE
local lua_tikz3dtools = {}

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

-- Expressions deliberately run with the privileges of the LuaTeX process.
-- This package does not add a second security boundary on top of LuaTeX.
-- TeX/LuaTeX configuration (including shell-escape policy) remains the
-- authority for what the process itself is allowed to do.
--
-- The environment metatable is shared.  Per-simplex programmable styles can
-- execute hundreds of thousands of times in an animation; allocating fresh
-- __index/__newindex closures for every simplex is needless GC pressure.
local eval_env_metatable = {
    __index = function(_, key)
        local value = lua_tikz3dtools.objects[key]
        if value ~= nil then
            return value
        end

        value = rawget(_G, key)
        if value ~= nil then
            return value
        end

        if key == "tau" then
            return 2 * math.pi
        end

        return math[key]
    end,
    __newindex = _G,
}

local function make_eval_env(bindings)
    local env = {}
    for key, value in pairs(bindings or {}) do
        env[key] = value
    end
    return setmetatable(env, eval_env_metatable)
end

local function load_chunk(source, label, bindings)
    return load(source, label or "expression", "t", make_eval_env(bindings))
end

local function execute_chunk(chunk, label, source)
    local ok, result = pcall(chunk)
    if not ok then
        error(format_eval_error("Lua evaluation error", label, source, result), 0)
    end

    return result
end

local function evaluate_chunk(source, label, bindings)
    local chunk, syntax_err = load_chunk(source, label, bindings)

    if not chunk then
        error(format_eval_error("Lua syntax error", label, source, syntax_err), 0)
    end

    return execute_chunk(chunk, label, source)
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

local command_guard = function(_, fn) return fn() end

local function register_tex_cmd(name, func, args, protected)
    local public_name = name
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
        command_guard(public_name, function()
            func(table.unpack(values))
        end)
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
lua_tikz3dtools.named_surfaces = {}
lua_tikz3dtools.next_surface_patch_id = 0

local function shallow_copy(map)
    local out = {}
    for key, value in pairs(map) do out[key] = value end
    return out
end

local function scene_checkpoint()
    return {
        simplex_count = #lua_tikz3dtools.simplices,
        light_count = #lua_tikz3dtools.lights,
        objects = shallow_copy(lua_tikz3dtools.objects),
        named_surfaces = shallow_copy(lua_tikz3dtools.named_surfaces),
        next_surface_patch_id = lua_tikz3dtools.next_surface_patch_id,
    }
end

local function restore_scene_checkpoint(checkpoint)
    while #lua_tikz3dtools.simplices > checkpoint.simplex_count do
        lua_tikz3dtools.simplices[#lua_tikz3dtools.simplices] = nil
    end
    while #lua_tikz3dtools.lights > checkpoint.light_count do
        lua_tikz3dtools.lights[#lua_tikz3dtools.lights] = nil
    end
    lua_tikz3dtools.objects = checkpoint.objects
    lua_tikz3dtools.named_surfaces = checkpoint.named_surfaces
    lua_tikz3dtools.next_surface_patch_id = checkpoint.next_surface_patch_id
end

-- Public scene commands are transactional.  A malformed object or a local
-- numerical failure removes only what that command was constructing; the TeX
-- job and the already-built scene continue.
local function run_scene_command(label, fn)
    local checkpoint = scene_checkpoint()
    local ok, result = xpcall(fn, function(err) return err end)
    if not ok then
        restore_scene_checkpoint(checkpoint)
        Recovery.log(label .. " skipped", result)
        return nil
    end
    return result
end

command_guard = run_scene_command

-- ================================================================
-- Expression evaluators (unrestricted Lua)
-- ================================================================

local function body_expression(str, label, bindings)
    return evaluate_chunk(str, label, bindings)
end

local function object_expression(str, label, bindings)
    local expression_source = ("return %s"):format(str)
    local chunk = load_chunk(expression_source, label, bindings)
    if chunk then
        return execute_chunk(chunk, label, str)
    end

    return body_expression(str, label, bindings)
end

-- TikZ style options are Lua bodies, just like filters.  A body is evaluated
-- once for each final render simplex with A/B/C bound directly in its
-- environment.  Triangle bodies additionally receive lighting and normal.
-- The usual simple case remains `return "fill=red"` / `return "draw=black"`.
--
-- Historical option syntax remains accepted:
--   * a literal TikZ option string such as `draw=black,line width=.4pt`,
--   * a Lua string expression,
--   * the old surface form `return function(lighting, triangle, normal) ... end`.
-- If a body returns a function, that function is immediately evaluated with
-- the legacy positional arguments for that simplex.
local function compile_style_body(body, label)
    local source = (
        "return function(env)\n"
        .. "  local _ENV = env\n"
        .. "%s\n"
        .. "end"
    ):format(body)

    local chunk, syntax_err = load(source, label, "t", make_eval_env())
    if not chunk then
        error(format_eval_error("Lua syntax error", label, body, syntax_err), 0)
    end
    local evaluator = execute_chunk(chunk, label, body)

    return function(bindings, ...)
        local ok, result = pcall(evaluator, make_eval_env(bindings))
        if not ok then
            error(format_eval_error("Lua function error", label, body, result), 0)
        end

        -- Backwards compatibility with the historical surface callback form.
        if type(result) == "function" then
            ok, result = pcall(result, ...)
            if not ok then
                error(format_eval_error("Lua function error", label, body, result), 0)
            end
        end

        if type(result) ~= "string" then
            error(label .. " must return a string", 0)
        end
        return result
    end
end

local function style_options_expression(str, label, bindings)
    label = label or "style options expression"

    if type(str) == "function" then
        local fn = wrap_user_function(str, label, "<function>")
        return function(_, ...)
            local result = fn(...)
            if type(result) ~= "string" then
                error(label .. " callback must return a string", 0)
            end
            return result
        end
    end
    if type(str) ~= "string" or str == "" then
        return ""
    end

    local trimmed = str:match("^%s*(.-)%s*$") or ""
    if trimmed == "" then
        return ""
    end

    -- Keep the historical body/expression heuristic unchanged.  Only failure
    -- handling differs: malformed style code falls back to an empty style.
    if starts_with_statement(trimmed)
        or trimmed:match("%f[%a]return%f[%A]")
    then
        local ok, compiled = pcall(compile_style_body, str, label)
        if not ok then
            Recovery.log(label .. " recovered", tostring(compiled) .. "; using empty options")
            return ""
        end
        return compiled
    end

    local expression_source = ("return %s"):format(str)
    local chunk = load_chunk(expression_source, label, bindings)
    if chunk then
        local ok, value = pcall(execute_chunk, chunk, label, str)
        if not ok then
            Recovery.log(label .. " recovered", tostring(value) .. "; using empty options")
            return ""
        end
        if type(value) == "string" then
            return value
        end
        if type(value) == "function" then
            local fn = wrap_user_function(value, label, str)
            return function(_, ...)
                local result = fn(...)
                if type(result) ~= "string" then
                    error(label .. " callback must return a string", 0)
                end
                return result
            end
        end
        Recovery.log(label .. " recovered", "style was neither a string nor function; using empty options")
        return ""
    end

    -- A non-Lua value such as `draw=black` is an ordinary literal TikZ option
    -- string, matching the historical behaviour of these keys.
    return str
end

local function fill_options_expression(str, label, bindings)
    return style_options_expression(str, label or "fill options expression", bindings)
end

local function draw_options_expression(str, label, bindings)
    return style_options_expression(str, label or "draw options expression", bindings)
end

local function sampled_string_function(str, label, signature, bindings)
    local fn = evaluate_chunk(("return function(%s) %s end"):format(signature, str), label, bindings)
    return function(...)
        local ok, result = pcall(fn, ...)
        if not ok then
            Recovery.log_once(
                "sample:" .. tostring(label) .. ":" .. tostring(str),
                label and (label .. " sample omitted") or "parametric sample omitted",
                format_eval_error("Lua function error", label, str, result)
            )
            return nil
        end
        return result
    end
end

local function single_string_function(str, label, bindings)
    return sampled_string_function(str, label, "u", bindings)
end

local function double_string_function(str, label, bindings)
    return sampled_string_function(str, label, "u,v", bindings)
end

local function triple_string_function(str, label, bindings)
    return sampled_string_function(str, label, "u,v,w", bindings)
end

local function is_finite_number(value)
    return type(value) == "number"
        and value == value
        and value ~= math.huge
        and value ~= -math.huge
end

-- Scene geometry is expressed in caller/drawing coordinates, so geometric
-- tolerances scale with the local simplex instead of assuming unit-sized
-- diagrams.  Parameter-space tolerances (for t in [0,1]) remain dimensionless
-- and are intentionally separate.
local SCENE_REL_EPS = 1e-12
local SCENE_PARTITION_REL_EPS = 1e-10
local SCENE_MERGE_REL_EPS = 1e-8
local MIN_SCENE_SCALE = 1e-150

local function simplex_length_scale(simplex)
    local scale = MIN_SCENE_SCALE
    if not simplex then return scale end
    for i = 1, #simplex do
        local a = Vector:_new(simplex[i])
        for j = i + 1, #simplex do
            local b = Vector:_new(simplex[j])
            scale = math.max(scale, a:_hdistance(b))
        end
    end
    return scale
end

local function point_pair_epsilon(a, b, relative)
    local coordinate_scale = MIN_SCENE_SCALE
    for _, point in ipairs{a, b} do
        for i = 1, #point - 1 do
            coordinate_scale = math.max(coordinate_scale, math.abs(point[i]))
        end
    end
    return (relative or SCENE_REL_EPS) * coordinate_scale
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

local function resolve_transformation(source, label)
    local ok, value = pcall(object_expression, source, label or "transformation")
    if ok and is_finite_numeric_matrix(value) then
        return value
    end
    Recovery.log_once(
        "transformation-fallback:" .. tostring(label or "transformation"),
        (label or "transformation") .. " recovered",
        ok and "invalid/non-finite matrix; using identity" or value
    )
    return Matrix.identity()
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
        Recovery.log_once(
            "projection:point:" .. tostring(label),
            (label or "point") .. " removed",
            "projection failed: " .. tostring(projected)
        )
        return nil
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
        Recovery.log_once(
            "projection:simplex:" .. tostring(label),
            (label or "simplex") .. " removed",
            "projection failed: " .. tostring(projected)
        )
        return nil
    end

    if is_finite_simplex(projected) then
        return projected
    end

    return nil
end

-- Adaptive refinement is judged in the drawing plane, after the same
-- transformation/projective divide used for the rendered simplex.  The raw
-- 3D sample is still retained by Adaptive and is what is ultimately emitted.
-- Keeping the metric here, rather than in the numerical mesher, prevents the
-- mesher from depending on Scene/Matrix internals while making animated view
-- or object transformations participate in refinement.
local function adaptive_screen_point(point, transformation, label)
    local projected = project_point(point, transformation, label)
    if not projected then
        return nil
    end

    return Vector:new{projected[1], projected[2], 0, 1}
end

local function simplex_matrix_is_degenerate(simplex, rows, relative)
    relative = relative or SCENE_REL_EPS
    if getmetatable(simplex) ~= Matrix or #simplex ~= rows then
        return false
    end

    local scale = simplex_length_scale(simplex)
    if rows == 2 then
        local A = Vector:_new(simplex[1])
        local B = Vector:_new(simplex[2])
        return A:_hdistance(B) <= point_pair_epsilon(A, B, relative)
    end

    if rows == 3 then
        local A = Vector:_new(simplex[1])
        local B = Vector:_new(simplex[2])
        local C = Vector:_new(simplex[3])
        local area_twice = (B:_hsub(A)):_hcross(C:_hsub(A)):hnorm()
        return area_twice <= relative * scale * scale
    end

    return false
end

local function simplex_entry_is_degenerate(entry)
    if entry.type == "line segment" then
        return simplex_matrix_is_degenerate(entry.simplex, 2)
    end
    if entry.type == "triangle" then
        return simplex_matrix_is_degenerate(entry.simplex, 3)
    end
    return false
end

local function push_simplex(entry)
    if entry.simplex
        and is_finite_simplex(entry.simplex)
        and not simplex_entry_is_degenerate(entry)
    then
        table.insert(lua_tikz3dtools.simplices, entry)
        return true
    end

    return false
end

local function is_nonempty_string(value)
    return type(value) == "string" and value ~= ""
end


local function normalize_partition_side(value)
    value = tostring(value or "both"):lower():gsub("%s+", "")
    if value ~= "positive" and value ~= "negative" and value ~= "both" then
        return "both"
    end
    return value
end

local function register_named_surface(name)
    if not is_nonempty_string(name) then return nil end
    if lua_tikz3dtools.named_surfaces[name] ~= nil then
        Recovery.log_once(
            "duplicate-surface-name:" .. name,
            "surface rendered unnamed",
            ("name '%s' was already in use"):format(name)
        )
        return nil
    end

    local surface = {
        name = name,
        triangles = {},
        tree = nil,
    }
    lua_tikz3dtools.named_surfaces[name] = surface
    return surface
end

local function resolve_partition_specs(source, owner_type)
    if not is_nonempty_string(source) then return nil end

    local ok, value = pcall(body_expression, source, "partition by")
    if not ok or type(value) ~= "table" then
        Recovery.log_once(
            "partition-spec-invalid",
            "partition specification ignored",
            ok and "partition_by did not return a table" or value
        )
        return nil
    end

    local specs = {}
    for i, item in ipairs(value) do
        if type(item) ~= "table" then
            Recovery.log_once("partition-entry-invalid", "partition entry ignored", "one or more entries were not keyed tables")
            goto continue_partition_item
        end
        if type(item.surface) ~= "string" or item.surface == "" then
            Recovery.log_once("partition-entry-no-surface", "partition entry ignored", "one or more entries did not name a surface")
            goto continue_partition_item
        end

        local surface = lua_tikz3dtools.named_surfaces[item.surface]
        if surface == nil then
            Recovery.log_once(
                "partition-entry-unknown-surface:" .. item.surface,
                "partition entry ignored",
                ("unknown surface '%s'"):format(item.surface)
            )
            goto continue_partition_item
        end

        local intersection_options = item.intersection_options or item.intersectionoptions
        if intersection_options ~= nil and type(intersection_options) ~= "string" then
            Recovery.log_once(
                "partition-intersection-style-invalid",
                "partition intersection style ignored",
                "one or more entries did not provide a TikZ option string"
            )
            intersection_options = nil
        end
        if owner_type == "curve" and intersection_options ~= nil then
            Recovery.log_once(
                "partition-intersection-style-curve",
                "partition intersection style ignored",
                "curve-surface intersections are zero-dimensional"
            )
            intersection_options = nil
        end

        specs[#specs + 1] = {
            surface = surface,
            keep = normalize_partition_side(item.keep),
            intersection_options = intersection_options or "",
        }
        ::continue_partition_item::
    end
    return (#specs > 0) and specs or nil
end

local function copy_record_with_simplex(source, simplex)
    local out = {}
    for k, v in pairs(source) do
        if k ~= "simplex" and k ~= "bbox2" and k ~= "named_partitions" then out[k] = v end
    end
    out.simplex = simplex
    out.bbox2 = nil
    if out.type == "triangle" then out.shading_normal = nil end
    return out
end

local function partition_piece_representative(record)
    if record.type == "line segment" then
        return Vector:_new(record.simplex[1]):_add(Vector:_new(record.simplex[2])):_scale(0.5)
    end
    return Vector:_new(record.simplex[1]):_add(Vector:_new(record.simplex[2]))
        :_add(Vector:_new(record.simplex[3])):_scale(1/3)
end

local function oriented_triangle_side(surface_record, point)
    local triangle = surface_record.simplex
    local A = Vector:_new(triangle[1])
    local normal = surface_record.surface_partition_normal
    if normal == nil then
        local B = Vector:_new(triangle[2])
        local C = Vector:_new(triangle[3])
        normal = (B:_hsub(A)):_hcross(C:_hsub(A))
        local scale = simplex_length_scale(triangle)
        if normal:hnorm() <= SCENE_REL_EPS * scale * scale then return 0 end
        normal = normal:hnormalize()
    end
    return Vector:_new(point):_hsub(A):_hinner(normal)
end

local function named_surface_tree(surface)
    if surface.tree ~= nil then return surface.tree end
    local entries = {}
    for i, record in ipairs(surface.triangles) do
        entries[#entries + 1] = {index = i, box = record.simplex:get_bbox3()}
    end
    surface.tree = AABBTree.build(entries, 3)
    return surface.tree
end

local function partition_side_is_kept(value, keep, scale)
    local eps = SCENE_PARTITION_REL_EPS * math.max(scale or 0, MIN_SCENE_SCALE)
    if keep == "both" then return true end
    if keep == "positive" then return value >= -eps end
    return value <= eps
end

local function ensure_surface_patch_id(record)
    if record.surface_patch_id == nil then
        lua_tikz3dtools.next_surface_patch_id = lua_tikz3dtools.next_surface_patch_id + 1
        record.surface_patch_id = lua_tikz3dtools.next_surface_patch_id
    end
    return record.surface_patch_id
end

local function normalize_partition_parts(parts, expected_rows)
    if type(parts) ~= "table" or #parts == 0 then return nil end
    local out = {}
    for _, part in ipairs(parts) do
        if getmetatable(part) == Matrix
            and #part == expected_rows
            and not simplex_matrix_is_degenerate(part, expected_rows, 1e-10)
        then
            out[#out + 1] = part
        end
    end
    return (#out > 0) and out or nil
end

local function intersection_highlight(record, surface_record, spec, intersection)
    if record.type ~= "triangle" or spec.intersection_options == "" or intersection == nil then
        return nil
    end

    local source_patch = ensure_surface_patch_id(record)
    local surface_patch = ensure_surface_patch_id(surface_record)
    local support_ids = {
        [source_patch] = true,
        [surface_patch] = true,
    }

    return {
        simplex = intersection,
        drawoptions = spec.intersection_options,
        type = "line segment",
        filter = "return true",
        support_patch_ids = support_ids,
        intersection_curve = true,
    }
end

local function partition_record_against_display_triangle(record, spec, surface_record)
    if record.type ~= "triangle" and record.type ~= "line segment" then
        return {record}, {}
    end

    local parts, intersection
    if record.type == "triangle" then
        intersection = Geometry.htriangle_triangle_intersections(record.simplex, surface_record.simplex)
        parts = Geometry.hpartition_triangle_by_triangle(record.simplex, surface_record.simplex)
    else
        intersection = Geometry.hline_segment_triangle_intersection(record.simplex, surface_record.simplex)
        parts = Geometry.hpartition_line_segment_by_triangle(record.simplex, surface_record.simplex)
    end

    local expected_rows = (record.type == "triangle") and 3 or 2
    local normalized = normalize_partition_parts(parts, expected_rows)
    local highlights = {}
    local highlight = intersection_highlight(record, surface_record, spec, intersection)
    if highlight ~= nil then highlights[#highlights + 1] = highlight end

    -- Merely touching an edge/vertex does not create pieces.  In that case the
    -- simplex is left untouched; partition_by is deliberately local and does
    -- not classify geometry that was not actually subdivided.
    if normalized == nil then
        return {record}, highlights
    end

    local selected = {}
    for _, simplex in ipairs(normalized) do
        local piece = copy_record_with_simplex(record, simplex)
        local side = oriented_triangle_side(surface_record, partition_piece_representative(piece))
        if partition_side_is_kept(
            side, spec.keep, simplex_length_scale(surface_record.simplex))
        then
            selected[#selected + 1] = piece
        end
    end
    return selected, highlights
end

local function partition_record_once(record, spec)
    local pieces = {record}
    local highlights = {}

    -- The named surface *is* its displayed simplicial surface.  Use its 3D
    -- BVH only as a conservative broad phase; exact intersection decisions
    -- remain in the existing Geometry line/triangle and triangle/triangle code.
    local candidate_indices = {}
    named_surface_tree(spec.surface):query(record.simplex:get_bbox3(), function(index)
        candidate_indices[#candidate_indices + 1] = index
    end)
    table.sort(candidate_indices)

    for _, index in ipairs(candidate_indices) do
        local surface_record = spec.surface.triangles[index]
        local next_pieces = {}
        for _, piece in ipairs(pieces) do
            local new_pieces, new_highlights =
                partition_record_against_display_triangle(piece, spec, surface_record)
            for _, p in ipairs(new_pieces) do next_pieces[#next_pieces + 1] = p end
            for _, h in ipairs(new_highlights) do highlights[#highlights + 1] = h end
        end
        pieces = next_pieces
        if #pieces == 0 then break end
    end

    return pieces, highlights
end

local function same_segment(a, b)
    local a1, a2 = Vector:_new(a.simplex[1]), Vector:_new(a.simplex[2])
    local b1, b2 = Vector:_new(b.simplex[1]), Vector:_new(b.simplex[2])
    local eps = SCENE_MERGE_REL_EPS * math.max(
        simplex_length_scale(a.simplex),
        simplex_length_scale(b.simplex)
    )
    return (a1:_hdistance(b1) <= eps and a2:_hdistance(b2) <= eps)
        or (a1:_hdistance(b2) <= eps and a2:_hdistance(b1) <= eps)
end

local function expanded_bbox3(simplex, margin)
    local box = simplex:get_bbox3()
    local min_values = {}
    local max_values = {}
    for d = 1, 3 do
        min_values[d] = box.min[d] - margin
        max_values[d] = box.max[d] + margin
    end
    return {min = min_values, max = max_values}
end

local function merge_support_ids(target, source)
    target.support_patch_ids = target.support_patch_ids or {}
    for id in pairs(source.support_patch_ids or {}) do
        target.support_patch_ids[id] = true
    end
end

local function deduplicate_highlights(highlights)
    if #highlights <= 1 then return highlights end

    local entries = {}
    for i, highlight in ipairs(highlights) do
        local margin = SCENE_MERGE_REL_EPS
            * simplex_length_scale(highlight.simplex)
        entries[#entries + 1] = {
            index = i,
            box = expanded_bbox3(highlight.simplex, margin),
        }
    end

    local neighbors = {}
    for i = 1, #highlights do neighbors[i] = {} end
    AABBTree.build(entries, 3):foreach_overlapping_pair(function(i, j)
        neighbors[j][#neighbors[j] + 1] = i
    end)
    for i = 1, #neighbors do table.sort(neighbors[i]) end

    local representative = {}
    local output = {}
    for i, candidate in ipairs(highlights) do
        local matched_rep = nil
        local seen_rep = {}
        for _, previous in ipairs(neighbors[i]) do
            local rep = representative[previous]
            if rep ~= nil and not seen_rep[rep] then
                seen_rep[rep] = true
                local existing = highlights[rep]
                if existing.drawoptions == candidate.drawoptions
                    and same_segment(existing, candidate)
                then
                    matched_rep = rep
                    break
                end
            end
        end

        if matched_rep ~= nil then
            representative[i] = matched_rep
            merge_support_ids(highlights[matched_rep], candidate)
        else
            representative[i] = i
            output[#output + 1] = candidate
        end
    end

    return output
end

local function apply_named_surface_partitions(simplices)
    local result, highlights = {}, {}

    -- Rebuild each named surface from the triangles that actually survive its
    -- own earlier partition_by operations.  Because citations are required to
    -- refer to previously named surfaces, the cited display mesh is complete
    -- by the time a later object is processed.
    for _, surface in pairs(lua_tikz3dtools.named_surfaces) do
        surface.triangles = {}
        surface.tree = nil
    end

    local function register_display_piece(piece)
        local owner = piece.named_surface_owner
        if owner ~= nil and piece.type == "triangle" then
            owner.triangles[#owner.triangles + 1] = piece
            owner.tree = nil
        end
    end

    local function apply_remaining_specs_to_highlight(highlight, specs, start_index)
        local pieces = {highlight}
        for si = start_index, #specs do
            local spec = specs[si]
            local clipping_spec = {
                surface = spec.surface,
                keep = spec.keep,
                intersection_options = "",
            }
            local next_pieces = {}
            for _, piece in ipairs(pieces) do
                local clipped = partition_record_once(piece, clipping_spec)
                for _, p in ipairs(clipped) do next_pieces[#next_pieces + 1] = p end
            end
            pieces = next_pieces
            if #pieces == 0 then break end
        end
        return pieces
    end

    for _, original in ipairs(simplices) do
        local specs = original.named_partitions
        local pieces = {original}

        if specs ~= nil then
            for si, spec in ipairs(specs) do
                local next_pieces = {}
                for _, piece in ipairs(pieces) do
                    local new_pieces, new_highlights = partition_record_once(piece, spec)
                    for _, p in ipairs(new_pieces) do next_pieces[#next_pieces + 1] = p end
                    for _, h in ipairs(new_highlights) do
                        local clipped_highlights = apply_remaining_specs_to_highlight(h, specs, si + 1)
                        for _, ch in ipairs(clipped_highlights) do highlights[#highlights + 1] = ch end
                    end
                end
                pieces = next_pieces
                if #pieces == 0 then break end
            end
        end

        for _, piece in ipairs(pieces) do
            piece.named_partitions = nil
            register_display_piece(piece)
            result[#result + 1] = piece
        end
    end

    highlights = deduplicate_highlights(highlights)
    for _, h in ipairs(highlights) do
        h.named_partitions = nil
        result[#result + 1] = h
    end
    return result
end

local DEFAULT_UV_ARROW_SCALE = 0.5
local DEFAULT_ARROW_TIP_LENGTH = 0.15
local DEFAULT_ARROW_TIP_RADIUS = 0.05
local DEFAULT_ARROW_TIP_FACETS = 6
local DEFAULT_ARROW_TIP_OPTIONS = "fill=black,draw=black"

local function resolve_arrow_tip_spec(source, label)
    label = label or "arrow tip"
    if source == nil or source == "" then
        return nil
    end

    local ok, value = pcall(object_expression, source, label)
    if not ok or type(value) ~= "table" then
        Recovery.log_once(
            "arrow-tip-spec:" .. tostring(label),
            label .. " omitted",
            ok and "specification was not a table" or value
        )
        return nil
    end

    local length = value.length
    if not (is_finite_number(length) and length > 0) then
        length = DEFAULT_ARROW_TIP_LENGTH
    end

    local radius = value.radius
    if not (is_finite_number(radius) and radius > 0) then
        radius = DEFAULT_ARROW_TIP_RADIUS
    end

    local facets = value.facets
    if not (is_finite_number(facets) and facets >= 3 and facets == math.floor(facets)) then
        facets = DEFAULT_ARROW_TIP_FACETS
    end

    local options = value.options
    if options == nil then options = DEFAULT_ARROW_TIP_OPTIONS end
    if type(options) ~= "string" and type(options) ~= "function" then
        options = DEFAULT_ARROW_TIP_OPTIONS
    elseif type(options) == "function" then
        options = wrap_user_function(options, label .. ".options", source)
    end

    return {
        length = length,
        radius = radius,
        facets = facets,
        options = options,
    }
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
    if start_point and stop_point
        and start_point:_hdistance(stop_point)
            > point_pair_epsilon(start_point, stop_point)
    then
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
    if length <= point_pair_epsilon(tip_point, tail_point) then
        return
    end

    local tip_scale = math.min(scale or DEFAULT_UV_ARROW_SCALE, length)
    if tip_scale <= SCENE_REL_EPS then
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

            if P and Q and P:_hdistance(Q) > point_pair_epsilon(P, Q) then
                local arrowscale = segment.arrowscale
                if arrowscale == nil then
                    arrowscale = DEFAULT_UV_ARROW_SCALE
                end
                if not (is_finite_number(arrowscale) and arrowscale > 0) then
                    arrowscale = DEFAULT_UV_ARROW_SCALE
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

local function next_surface_patch_id()
    lua_tikz3dtools.next_surface_patch_id = lua_tikz3dtools.next_surface_patch_id + 1
    return lua_tikz3dtools.next_surface_patch_id
end

local function segment_point_at(segment, t)
    local P0 = Vector:_new(segment[1])
    local P1 = Vector:_new(segment[2])
    return P0:_scale(1 - t):_add(P1:_scale(t))
end

local function sorted_unique_parameters(values)
    table.sort(values)
    local result = {}
    local eps = 1e-10
    for _, value in ipairs(values) do
        value = math.max(0, math.min(1, value))
        if #result == 0 or math.abs(value - result[#result]) > eps then
            result[#result + 1] = value
        end
    end
    return result
end

-- Convert one UV line segment into independent 3D line fragments supported by
-- the surface mesh.  We first find every triangle interval on the original
-- parameter segment, then split at all interval endpoints.  Each atomic
-- fragment therefore has a definitive set of supporting surface patches.
-- Shared mesh edges naturally produce multiple support ids instead of duplicate
-- curve geometry.
local function append_supported_surface_segment(segment, surface_patches, patch_tree, named_partitions)
    local uv_segment = segment.simplex
    local intervals = {}
    local boundaries = {}
    local eps = 1e-10

    local candidate_indices = {}
    patch_tree:query(uv_segment:get_bbox2(), function(index)
        candidate_indices[#candidate_indices + 1] = index
    end)
    table.sort(candidate_indices)

    for _, index in ipairs(candidate_indices) do
        local patch = surface_patches[index]
        local t0, t1 = Geometry.hclip_line_segment_to_triangle_interval(
            uv_segment,
            patch.uv_triangle
        )
        if t0 ~= nil then
            intervals[#intervals + 1] = {
                t0 = t0,
                t1 = t1,
                patch = patch,
            }
            boundaries[#boundaries + 1] = t0
            boundaries[#boundaries + 1] = t1
        end
    end

    if #intervals == 0 then
        return
    end

    boundaries = sorted_unique_parameters(boundaries)
    for i = 1, #boundaries - 1 do
        local t0 = boundaries[i]
        local t1 = boundaries[i + 1]
        if t1 - t0 > eps then
            local midpoint = (t0 + t1) / 2
            local primary_patch = nil
            local support_patch_ids = {}

            for _, interval in ipairs(intervals) do
                if interval.t0 - eps <= midpoint and midpoint <= interval.t1 + eps then
                    local patch = interval.patch
                    support_patch_ids[patch.id] = true
                    if primary_patch == nil or patch.id < primary_patch.id then
                        primary_patch = patch
                    end
                end
            end

            if primary_patch ~= nil then
                local uv_start = segment_point_at(uv_segment, t0)
                local uv_stop = segment_point_at(uv_segment, t1)
                local start_bary = Geometry.hpoint_triangle_barycentric(
                    uv_start,
                    primary_patch.uv_triangle
                )
                local stop_bary = Geometry.hpoint_triangle_barycentric(
                    uv_stop,
                    primary_patch.uv_triangle
                )

                if start_bary ~= nil and stop_bary ~= nil then
                    local start_point = Geometry.hpoint_from_triangle_barycentric(
                        primary_patch.simplex,
                        start_bary
                    )
                    local stop_point = Geometry.hpoint_from_triangle_barycentric(
                        primary_patch.simplex,
                        stop_bary
                    )
                    if start_point:_hdistance(stop_point)
                        > point_pair_epsilon(start_point, stop_point)
                    then
                        push_simplex({
                            simplex = Matrix:_new{start_point, stop_point},
                            drawoptions = segment.drawoptions or "",
                            type = "line segment",
                            filter = "return true",
                            support_patch_ids = support_patch_ids,
                            surface_curve = true,
                            named_partitions = named_partitions,
                        })
                    end
                end
            end
        end
    end
end

local function append_supported_surface_curves(uv_segments, surface_patches, named_partitions)
    if uv_segments == nil or #surface_patches == 0 then
        return
    end

    local entries = {}
    for i, patch in ipairs(surface_patches) do
        entries[#entries + 1] = {
            index = i,
            box = patch.uv_triangle:get_bbox2(),
        }
    end
    local patch_tree = AABBTree.build(entries, 2)

    for _, segment in ipairs(uv_segments) do
        append_supported_surface_segment(
            segment,
            surface_patches,
            patch_tree,
            named_partitions
        )
    end
end

local function triangle_shading_normal(simplex)
    local A = Vector:_new(simplex[1])
    local B = Vector:_new(simplex[2])
    local C = Vector:_new(simplex[3])
    local normal = (B:_hsub(A)):_hcross(C:_hsub(A))

    local scale = simplex_length_scale(simplex)
    if normal:hnorm() <= SCENE_REL_EPS * scale * scale then
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

local function resolve_axis_params(params_src)
    local ok, value = pcall(body_expression, params_src)
    if ok and value and getmetatable(value) == Vector
        and is_finite_number(value[1])
        and is_finite_number(value[2])
        and is_finite_number(value[3])
        and value[3] >= 2
        and value[3] == math.floor(value[3])
    then
        return value[1], value[2], value[3]
    end

    Recovery.log_once(
        "axis-params-fallback",
        "axis parameters recovered",
        ok and "invalid axis parameter triplet; using {0,1,10}" or value
    )
    return 0, 1, 10
end

local function resolve_adaptive_spec(adaptive_src, label)
    if not is_nonempty_string(adaptive_src) then
        return nil
    end

    local ok, spec = pcall(object_expression, adaptive_src, label or "adaptive")
    if not ok or type(spec) ~= "table" then
        Recovery.log_once(
            "adaptive-spec-disabled:" .. tostring(label or "adaptive"),
            (label or "adaptive") .. " recovered",
            ok and "specification was not a table; adaptive refinement disabled" or spec
        )
        return nil
    end
    return spec
end

-- Return one sample from an evenly spaced closed interval.  Compute interior
-- samples by interpolation, but preserve the caller's exact endpoints.  This
-- avoids floating-point roundoff producing a final sample just outside the
-- declared parameter domain.
local function axis_sample(start_value, stop_value, samples, index)
    if index == 0 then
        return start_value
    elseif index == samples - 1 then
        return stop_value
    end

    return start_value
        + (stop_value - start_value) * (index / (samples - 1))
end

local function uv_oriented_surface_normal(simplex, uv_points)
    local A = Vector:_new(simplex[1])
    local B = Vector:_new(simplex[2])
    local C = Vector:_new(simplex[3])
    local normal = (B:_hsub(A)):_hcross(C:_hsub(A))
    local surface_scale = simplex_length_scale(simplex)
    if normal:hnorm() <= SCENE_REL_EPS * surface_scale * surface_scale then
        return nil
    end

    local U1, U2, U3 = uv_points[1], uv_points[2], uv_points[3]
    local du1, dv1 = U2[1] - U1[1], U2[2] - U1[2]
    local du2, dv2 = U3[1] - U1[1], U3[2] - U1[2]
    local orientation = du1 * dv2 - dv1 * du2
    local uv_scale = math.max(
        math.sqrt(du1 * du1 + dv1 * dv1),
        math.sqrt(du2 * du2 + dv2 * dv2),
        MIN_SCENE_SCALE
    )
    if math.abs(orientation) <= SCENE_REL_EPS * uv_scale * uv_scale then
        return nil
    end
    if orientation < 0 then normal = normal:_hscale(-1) end
    return normal:hnormalize()
end

-- Forward declaration: arrow surfaces are built with the solid tessellator.
local append_solid

local function append_surface(hash)
    local ustart, ustop, usamples = resolve_axis_params(hash.uparams)
    local vstart, vstop, vsamples = resolve_axis_params(hash.vparams)
    local transformation = resolve_transformation(hash.transformation)
    local f              = double_string_function(hash.v)
    local filloptions    = fill_options_expression(hash.filloptions, "surface fill options")
    local filter         = hash.filter
    local named_partitions = resolve_partition_specs(hash.partitionby, "surface")
    local named_surface = register_named_surface(hash.name)
    local adaptive = resolve_adaptive_spec(hash.adaptive, "surface adaptive")
    local uv_curve_segments
    local surface_patches = {}

    assert(usamples and usamples >= 2, "usamples must be >= 2, got: " .. tostring(usamples))
    assert(vsamples and vsamples >= 2, "vsamples must be >= 2, got: " .. tostring(vsamples))

    local function parametric_surface(u, v)
        return f(u, v)
    end

    if is_nonempty_string(hash.curve) then
        uv_curve_segments = explicit_uv_curve_segments(hash.curve)
    end

    local function append_surface_triangle(points, uv_points)
        if Geometry.hpoint_point_intersecting(points[1], points[2])
            or Geometry.hpoint_point_intersecting(points[2], points[3])
            or Geometry.hpoint_point_intersecting(points[1], points[3])
        then
            return
        end

        local simplex = project_simplex(Matrix:_new(points), transformation)
        if simplex == nil then
            return
        end

        local patch_id = next_surface_patch_id()
        local record = {
            simplex = simplex,
            filloptions = filloptions,
            type = "triangle",
            filter = filter,
            shading_normal = triangle_shading_normal(simplex),
            surface_partition_normal = uv_oriented_surface_normal(simplex, uv_points),
            surface_patch_id = patch_id,
            surface_name = hash.name,
            named_surface_owner = named_surface,
            named_partitions = named_partitions,
        }
        if push_simplex(record) then
            surface_patches[#surface_patches + 1] = {
                id = patch_id,
                uv_triangle = Matrix:_new(uv_points),
                simplex = simplex,
            }
        end
    end

    if adaptive then
        local triangles, diagnostics = Adaptive.surface(
            parametric_surface,
            ustart, ustop, usamples,
            vstart, vstop, vsamples,
            adaptive,
            function(point)
                return adaptive_screen_point(
                    point, transformation, "adaptive surface sample")
            end
        )
        for _, triangle in ipairs(triangles) do
            append_surface_triangle(triangle.points, triangle.uv)
        end
        if diagnostics.exhausted then
            log_line(("Surface adaptive budget: %d tris, err=%.4g, tol=%.4g.")
                :format(diagnostics.triangles, diagnostics.worst_error, diagnostics.tolerance))
        end
        if diagnostics.unresolved > 0 then
            log_line(("Surface adaptive omitted: %d unresolved/nonfinite tris.")
                :format(diagnostics.unresolved))
        end
        if diagnostics.unmet > 0 and not diagnostics.exhausted then
            log_line(("Surface adaptive depth limit: %d tris over tol, err=%.4g.")
                :format(diagnostics.unmet, diagnostics.worst_error))
        end
        if diagnostics.domain_topology_safe == false then
            log_line(("Domain sewing fail-closed: %d constraints, %d boundary vertices.")
                :format(
                    diagnostics.constraints_failed or 0,
                    diagnostics.boundary_vertex_failures or 0
                ))
        end
    else
        for i = 0, usamples - 2 do
            local u0 = axis_sample(ustart, ustop, usamples, i)
            local u1 = axis_sample(ustart, ustop, usamples, i + 1)
            for j = 0, vsamples - 2 do
                local v0 = axis_sample(vstart, vstop, vsamples, j)
                local v1 = axis_sample(vstart, vstop, vsamples, j + 1)
                local A = parametric_surface(u0, v0)
                local B = parametric_surface(u1, v0)
                local C = parametric_surface(u1, v1)
                local D = parametric_surface(u0, v1)
                if A and B and C and D then
                    local uvA = Vector:_new{u0, v0, 1}
                    local uvB = Vector:_new{u1, v0, 1}
                    local uvC = Vector:_new{u1, v1, 1}
                    local uvD = Vector:_new{u0, v1, 1}

                    append_surface_triangle({A, B, C}, {uvA, uvB, uvC})
                    append_surface_triangle({A, D, C}, {uvA, uvD, uvC})
                end
            end
        end
    end

    -- Surface curves are independent line-segment simplices.  Their support
    -- metadata affects only the local support relationship in the occlusion
    -- graph; every other triangle remains a normal possible occluder.
    append_supported_surface_curves(uv_curve_segments, surface_patches, named_partitions)
end

local function arrow_basis(tip_point, base_point)
    local direction = tip_point:_hsub(base_point)
    if direction:hnorm() <= point_pair_epsilon(tip_point, base_point) then
        return nil
    end

    local U = direction:hnormalize()
    local V = U:horthogonal_vector():hnormalize()
    local W = U:_hcross(V):hnormalize()
    return U, V, W
end

local function fit_arrow_tip_to_segment(tip_point, neighbor_point, spec)
    if not (tip_point and neighbor_point and spec) then
        return nil
    end

    local direction = tip_point:_hsub(neighbor_point)
    local segment_length = direction:hnorm()
    if segment_length <= point_pair_epsilon(tip_point, neighbor_point) then
        return nil
    end

    -- A tip may not consume more than its terminal shaft segment.  If the
    -- requested length is longer, scale the whole tip uniformly so its base
    -- still meets the shaft instead of leaving a gap.
    local effective_length = math.min(spec.length, segment_length)
    local size_scale = effective_length / spec.length
    local U = direction:hnormalize()
    local base_point = tip_point:_hsub(U:_hscale(effective_length))

    return base_point, {
        length = effective_length,
        radius = spec.radius * size_scale,
        facets = spec.facets,
        options = spec.options,
    }
end

local function append_projected_arrow_tip(tip_point, base_point, spec, filter, named_partitions)
    if not (tip_point and base_point and spec) then
        return
    end

    local U, V, W = arrow_basis(tip_point, base_point)
    if not (U and V and W) then
        return
    end

    local facets = spec.facets
    local radius = spec.radius
    local ring = {}
    for i = 0, facets - 1 do
        local angle = 2 * math.pi * i / facets
        ring[#ring + 1] = base_point
            :_hadd(V:_hscale(radius * math.cos(angle)))
            :_hadd(W:_hscale(radius * math.sin(angle)))
    end

    local function append_face(A, B, C)
        local triangle = Matrix:_new{A, B, C}
        push_simplex({
            simplex = triangle,
            filloptions = spec.options,
            type = "triangle",
            filter = filter,
            named_partitions = named_partitions,
            shading_normal = triangle_shading_normal(triangle),
        })
    end

    for i = 1, facets do
        local j = i % facets + 1
        -- With V,W chosen around +U, this winding gives outward-facing side
        -- normals and a base normal facing away from the apex.
        append_face(tip_point, ring[i], ring[j])
        append_face(base_point, ring[j], ring[i])
    end
end

local function append_triangle(hash)
    local transformation = resolve_transformation(hash.transformation)
    local filter         = hash.filter
    local filloptions    = fill_options_expression(hash.filloptions, "triangle fill options")
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
    local transformation = resolve_transformation(hash.transformation)
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
    if v == nil or getmetatable(v) ~= Vector or not is_finite_simplex(v) then
        Recovery.log_once("light-invalid", "light omitted", "light v did not return a finite Vector")
        return
    end
    if v:hnorm() == 0 then
        Recovery.log_once("light-zero", "light omitted", "light direction was zero")
        return
    end
    lua_tikz3dtools.lights[#lua_tikz3dtools.lights + 1] = v
end


local function append_curve(hash)
    local ustart, ustop, usamples = resolve_axis_params(hash.uparams)
    local transformation = resolve_transformation(hash.transformation)
    local f              = single_string_function(hash.v)
    local filter         = hash.filter
    local drawoptions    = draw_options_expression(hash.drawoptions, "curve draw options")
    local arrowtip       = resolve_arrow_tip_spec(hash.arrowtip, "curve arrow tip")
    local arrowtail      = resolve_arrow_tip_spec(hash.arrowtail, "curve arrow tail")
    local named_partitions = resolve_partition_specs(hash.partitionby, "curve")
    local adaptive       = resolve_adaptive_spec(hash.adaptive, "curve adaptive")

    assert(usamples and usamples >= 2, "usamples must be >= 2, got: " .. tostring(usamples))

    local function parametric_curve(u)
        return f(u)
    end

    local segments = {}
    local diagnostics
    if adaptive then
        segments, diagnostics = Adaptive.curve(
            parametric_curve, ustart, ustop, usamples, adaptive,
            function(point)
                return adaptive_screen_point(
                    point, transformation, "adaptive curve sample")
            end
        )
    else
        for i = 0, usamples - 2 do
            local u0 = axis_sample(ustart, ustop, usamples, i)
            local u1 = axis_sample(ustart, ustop, usamples, i + 1)
            segments[#segments + 1] = {
                a = {u = u0, point = parametric_curve(u0), valid = true},
                b = {u = u1, point = parametric_curve(u1), valid = true},
            }
        end
    end

    for i, segment in ipairs(segments) do
        local A = segment.a.point
        local B = segment.b.point
        if not segment.unsafe
            and segment.a.valid ~= false and segment.b.valid ~= false and A and B
        then
            local simplex = project_simplex(
                Matrix:_new{A, B},
                transformation
            )
            if simplex then
                local original_start = Vector:_new(simplex[1])
                local original_stop = Vector:_new(simplex[2])
                local shaft_start = original_start
                local shaft_stop = original_stop
                local tail_base, effective_tail
                local tip_base, effective_tip

                if i == 1 and arrowtail then
                    tail_base, effective_tail = fit_arrow_tip_to_segment(
                        original_start, original_stop, arrowtail)
                    if tail_base then shaft_start = tail_base end
                end

                if i == #segments and arrowtip then
                    tip_base, effective_tip = fit_arrow_tip_to_segment(
                        original_stop, original_start, arrowtip)
                    if tip_base then shaft_stop = tip_base end
                end

                local shaft_length = shaft_start:_hdistance(shaft_stop)
                local segment_length = original_start:_hdistance(original_stop)
                local tips_overlap = effective_tail and effective_tip
                    and effective_tail.length + effective_tip.length
                        >= segment_length - SCENE_REL_EPS * segment_length

                if shaft_length > SCENE_REL_EPS * segment_length and not tips_overlap then
                    push_simplex({
                        simplex      = Matrix:_new{shaft_start, shaft_stop},
                        drawoptions  = drawoptions,
                        type         = "line segment",
                        filter       = filter,
                        named_partitions = named_partitions
                    })
                end

                if effective_tail then
                    append_projected_arrow_tip(
                        original_start, tail_base, effective_tail,
                        filter, named_partitions)
                end

                if effective_tip then
                    append_projected_arrow_tip(
                        original_stop, tip_base, effective_tip,
                        filter, named_partitions)
                end
            end
        end
    end

    if diagnostics and diagnostics.exhausted then
        log_line(("Curve adaptive budget: %d segs, err=%.4g, tol=%.4g.")
            :format(diagnostics.segments, diagnostics.worst_error, diagnostics.tolerance))
    end
    if diagnostics and diagnostics.unresolved > 0 then
        log_line(("Curve adaptive omitted: %d unresolved/nonfinite segs.")
            :format(diagnostics.unresolved))
    end
    if diagnostics and diagnostics.unmet > 0 and not diagnostics.exhausted then
        log_line(("Curve adaptive depth limit: %d segs over tol, err=%.4g.")
            :format(diagnostics.unmet, diagnostics.worst_error))
    end
end

append_solid = function(hash)
    local ustart, ustop, usamples = resolve_axis_params(hash.uparams)
    local vstart, vstop, vsamples = resolve_axis_params(hash.vparams)
    local wstart, wstop, wsamples = resolve_axis_params(hash.wparams)
    local filloptions    = fill_options_expression(hash.filloptions, "solid fill options")
    local filter = hash.filter
    local named_partitions = resolve_partition_specs(hash.partitionby, "solid")
    local transformation = resolve_transformation(hash.transformation)
    local f = triple_string_function(hash.v)

    assert(usamples and usamples >= 2, "usamples must be >= 2, got: " .. tostring(usamples))
    assert(vsamples and vsamples >= 2, "vsamples must be >= 2, got: " .. tostring(vsamples))
    assert(wsamples and wsamples >= 2, "wsamples must be >= 2, got: " .. tostring(wsamples))

    local function parametric_solid(u, v, w)
        return f(u, v, w)
    end

    local function tessellate_face(
        fixed_var, fixed_val,
        s1_start, s1_stop, s1_count,
        s2_start, s2_stop, s2_count
    )
        for i = 0, s1_count - 2 do
            local s10 = axis_sample(s1_start, s1_stop, s1_count, i)
            local s11 = axis_sample(s1_start, s1_stop, s1_count, i + 1)
            for j = 0, s2_count - 2 do
                local s20 = axis_sample(s2_start, s2_stop, s2_count, j)
                local s21 = axis_sample(s2_start, s2_stop, s2_count, j + 1)
                local A, B, C, D
                if fixed_var == "u" then
                    A = parametric_solid(fixed_val, s10, s20)
                    B = parametric_solid(fixed_val, s11, s20)
                    C = parametric_solid(fixed_val, s11, s21)
                    D = parametric_solid(fixed_val, s10, s21)
                elseif fixed_var == "v" then
                    A = parametric_solid(s10, fixed_val, s20)
                    B = parametric_solid(s11, fixed_val, s20)
                    C = parametric_solid(s11, fixed_val, s21)
                    D = parametric_solid(s10, fixed_val, s21)
                elseif fixed_var == "w" then
                    A = parametric_solid(s10, s20, fixed_val)
                    B = parametric_solid(s11, s20, fixed_val)
                    C = parametric_solid(s11, s21, fixed_val)
                    D = parametric_solid(s10, s21, fixed_val)
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
                            shading_normal = triangle_shading_normal(simplex),
                            named_partitions = named_partitions
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
                            shading_normal = triangle_shading_normal(simplex),
                            named_partitions = named_partitions
                        })
                    end
                end
            end
        end
    end

    tessellate_face("u", ustart, vstart, vstop, vsamples, wstart, wstop, wsamples)
    tessellate_face("u", ustop,  vstart, vstop, vsamples, wstart, wstop, wsamples)
    tessellate_face("v", vstart, ustart, ustop, usamples, wstart, wstop, wsamples)
    tessellate_face("v", vstop,  ustart, ustop, usamples, wstart, wstop, wsamples)
    tessellate_face("w", wstart, ustart, ustop, usamples, vstart, vstop, vsamples)
    tessellate_face("w", wstop,  ustart, ustop, usamples, vstart, vstop, vsamples)
end

local function compile_filter(filter_body)
    local label = "filter"
    local source = (
        "return function(env)\n"
        .. "  local _ENV = env\n"
        .. "%s\n"
        .. "end"
    ):format(filter_body)

    local chunk, syntax_err = load(source, label, "t", make_eval_env())
    if not chunk then
        error(format_eval_error("Lua syntax error", label, filter_body, syntax_err), 0)
    end
    local evaluator = execute_chunk(chunk, label, filter_body)

    return function(bindings)
        local ok, result = pcall(evaluator, make_eval_env(bindings))
        if not ok then
            error(format_eval_error("Lua function error", label, filter_body, result), 0)
        end
        if type(result) ~= "boolean" then
            error("filter must return a boolean", 0)
        end
        return result
    end
end

local function apply_filters(simplices)
    local new_simplices = {}
    local compiled_filters = {}
    local broken_filters = {}

    for _, simplex in ipairs(simplices) do
        local bindings = {}

        if simplex.type == "line segment" then
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
        local filter_fn = compiled_filters[filter_body]
        if filter_fn == nil and not broken_filters[filter_body] then
            local ok, compiled = pcall(compile_filter, filter_body)
            if ok then
                filter_fn = compiled
                compiled_filters[filter_body] = compiled
            else
                broken_filters[filter_body] = true
                Recovery.log_once(
                    "filter-compile:" .. tostring(filter_body),
                    "filter dropped geometry",
                    "filter could not be compiled"
                )
            end
        end

        if filter_fn then
            local ok, keep = pcall(filter_fn, bindings)
            if ok and keep == true then
                new_simplices[#new_simplices + 1] = simplex
            elseif not ok then
                Recovery.log_once(
                    "filter-runtime:" .. tostring(filter_body),
                    "filter dropped simplex",
                    "filter failed"
                )
            elseif keep ~= false then
                Recovery.log_once(
                    "filter-result:" .. tostring(filter_body),
                    "filter dropped simplex",
                    "nonboolean result"
                )
            end
        end
    end

    return new_simplices
end

local function format_coordinate(value)
    if value == 0 then
        return "0"
    end
    return ("%.17g"):format(value)
end

local function reset_scene_render_state()
    lua_tikz3dtools.simplices = {}
    lua_tikz3dtools.lights = {}
    lua_tikz3dtools.named_surfaces = {}
    lua_tikz3dtools.next_surface_patch_id = 0
end

local function display_simplices()
    local output

    local function stage(label, current, fn)
        local ok, result = xpcall(fn, function(err) return err end)
        if ok and type(result) == "table" then
            return result
        end
        Recovery.log(label .. " fallback", ok and "stage returned no geometry" or result)
        return current
    end

    local function sanitize_bbox(simplices)
        local kept = {}
        for _, simplex in ipairs(simplices) do
            if simplex.type == "label" then
                kept[#kept + 1] = simplex
            elseif simplex.simplex and is_finite_simplex(simplex.simplex) then
                local ok, bbox = pcall(function() return simplex.simplex:get_bbox2() end)
                if ok and bbox then
                    simplex.bbox2 = bbox
                    kept[#kept + 1] = simplex
                else
                    Recovery.log_once(
                        "bbox:" .. tostring(bbox),
                        "simplex removed",
                        ok and "could not construct a screen-space bounding box" or bbox
                    )
                end
            else
                Recovery.log_once(
                    "bbox:nonfinite",
                    "simplex removed",
                    "non-finite geometry reached the renderer"
                )
            end
        end
        return kept
    end

    local function fallback_output()
        local lines = {}
        return {
            write = function(_, line) lines[#lines + 1] = line end,
            input = function()
                for _, line in ipairs(lines) do tex.sprint(line) end
                lines = {}
            end,
            discard = function() lines = {} end,
        }
    end

    local function new_output()
        local ok, result = pcall(VirtualFile.new)
        if ok and result then return result end
        Recovery.log("virtual input unavailable", result or "unknown failure; using direct TeX streaming")
        return fallback_output()
    end

    local function safe_render_line(simplex)
        local ok, line = xpcall(function()
            if simplex.type == "line segment" then
                local drawoptions = simplex.drawoptions or ""
                if type(drawoptions) == "function" then
                    local A = Vector:_new(simplex.simplex[1])
                    local B = Vector:_new(simplex.simplex[2])
                    drawoptions = drawoptions({A = A, B = B})
                end
                if type(drawoptions) ~= "string" then
                    error("draw options did not resolve to a string", 0)
                end
                return ("\\path[%s] (%s,%s) -- (%s,%s);")
                    :format(
                        drawoptions,
                        format_coordinate(simplex.simplex[1][1]),
                        format_coordinate(simplex.simplex[1][2]),
                        format_coordinate(simplex.simplex[2][1]),
                        format_coordinate(simplex.simplex[2][2])
                    )
            elseif simplex.type == "triangle" then
                local normal = simplex.shading_normal or triangle_shading_normal(simplex.simplex)
                local intensity = 0
                local usable_lights = 0

                if normal then
                    local total_intensity = 0
                    for _, light in ipairs(lua_tikz3dtools.lights) do
                        local light_ok, light_value = pcall(function()
                            local light_dir = light:hnormalize()
                            local facing_normal = light_facing_normal(normal, light_dir)
                            local cos_theta = facing_normal:hinner(light_dir)
                            if cos_theta > 1 then cos_theta = 1 end
                            if cos_theta < -1 then cos_theta = -1 end
                            local theta = math.deg(math.acos(cos_theta))
                            return 1 - theta / 90
                        end)
                        if light_ok and is_finite_number(light_value) then
                            total_intensity = total_intensity + light_value
                            usable_lights = usable_lights + 1
                        else
                            Recovery.log_once(
                                "render-light:" .. tostring(light_value),
                                "light ignored",
                                light_value
                            )
                        end
                    end
                    if usable_lights > 0 then
                        intensity = total_intensity / usable_lights
                        if intensity < 0 then intensity = 0 end
                        if intensity > 1 then intensity = 1 end
                    end
                end

                local filloptions = simplex.filloptions or ""
                if type(filloptions) == "function" then
                    local A = Vector:_new(simplex.simplex[1])
                    local B = Vector:_new(simplex.simplex[2])
                    local C = Vector:_new(simplex.simplex[3])
                    filloptions = filloptions(
                        {A = A, B = B, C = C, lighting = intensity, normal = normal},
                        intensity, simplex.simplex, normal
                    )
                end
                if type(filloptions) ~= "string" then
                    error("fill options did not resolve to a string", 0)
                end
                return ("\\path[%s] (%s,%s) -- (%s,%s) -- (%s,%s) -- cycle;")
                    :format(
                        filloptions,
                        format_coordinate(simplex.simplex[1][1]),
                        format_coordinate(simplex.simplex[1][2]),
                        format_coordinate(simplex.simplex[2][1]),
                        format_coordinate(simplex.simplex[2][2]),
                        format_coordinate(simplex.simplex[3][1]),
                        format_coordinate(simplex.simplex[3][2])
                    )
            end
            return nil
        end, function(err) return err end)

        if not ok then
            Recovery.log_once(
                "render-simplex:" .. tostring(simplex.type) .. ":" .. tostring(line),
                "style dropped simplex",
                simplex.type == "triangle" and "fill options failed" or "draw options failed"
            )
            return nil
        end
        return line
    end

    local function render()
        log_line(("Displaying %d simplices."):format(#lua_tikz3dtools.simplices))

        local before = #lua_tikz3dtools.simplices
        lua_tikz3dtools.simplices = stage(
            "named-surface partitioning",
            lua_tikz3dtools.simplices,
            function() return apply_named_surface_partitions(lua_tikz3dtools.simplices) end
        )
        if #lua_tikz3dtools.simplices ~= before then
            log_line(("Named-surface partitioning changed %d to %d simplices.")
                :format(before, #lua_tikz3dtools.simplices))
        end

        lua_tikz3dtools.simplices = sanitize_bbox(lua_tikz3dtools.simplices)

        before = #lua_tikz3dtools.simplices
        lua_tikz3dtools.simplices = stage(
            "simplicial partitioning",
            lua_tikz3dtools.simplices,
            function()
                return Geometry.partition_simplices_by_parents(
                    lua_tikz3dtools.simplices,
                    lua_tikz3dtools.simplices
                )
            end
        )
        if #lua_tikz3dtools.simplices ~= before then
            log_line(("Partitioning changed %d to %d simplices.")
                :format(before, #lua_tikz3dtools.simplices))
        end

        before = #lua_tikz3dtools.simplices
        lua_tikz3dtools.simplices = apply_filters(lua_tikz3dtools.simplices)
        if #lua_tikz3dtools.simplices ~= before then
            log_line(("Filtering changed %d to %d simplices.")
                :format(before, #lua_tikz3dtools.simplices))
        end

        lua_tikz3dtools.simplices = sanitize_bbox(lua_tikz3dtools.simplices)

        local render_simplices = stage(
            "occlusion sorting",
            lua_tikz3dtools.simplices,
            function() return Geometry.scc(lua_tikz3dtools.simplices) end
        )
        if #render_simplices ~= #lua_tikz3dtools.simplices then
            log_line(("Occlusion resolution changed %d to %d render simplices.")
                :format(#lua_tikz3dtools.simplices, #render_simplices))
        end

        output = new_output()
        local labels = {}
        for _, simplex in ipairs(render_simplices) do
            if simplex.type == "label" then
                labels[#labels + 1] = simplex
            else
                local line = safe_render_line(simplex)
                if line then
                    local ok, err = pcall(output.write, output, line)
                    if not ok then
                        Recovery.log_once("generated-path-omitted", "generated path omitted", err)
                    end
                end
            end
        end

        for _, simplex in ipairs(labels) do
            local ok, line = pcall(function()
                return ("\\node at (%s,%s) {%s};")
                    :format(
                        format_coordinate(simplex.simplex[1]),
                        format_coordinate(simplex.simplex[2]),
                        simplex.text
                    )
            end)
            if ok then
                local write_ok, write_err = pcall(output.write, output, line)
                if not write_ok then Recovery.log_once("label-write-omitted", "label omitted", write_err) end
            else
                Recovery.log_once("label-render-omitted", "label omitted", line)
            end
        end

        local input_ok, input_err = pcall(output.input, output)
        if not input_ok then
            Recovery.log("generated drawing input omitted", input_err)
            pcall(output.discard, output)
        end
    end

    local ok, err = xpcall(render, function(value) return value end)
    if not ok and output then pcall(output.discard, output) end
    if not ok then
        Recovery.log("display recovered", err)
    end
    reset_scene_render_state()
end

local function set_object(hash)
    local object = object_expression(hash.object)
    local name = object_expression(hash.name)

    assert(type(name) == "string" and name ~= "", "setobject.name must be a non-empty string")

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
    register_tex_cmd("appendsurface", function()
        append_surface{
            uparams        = get_macro_or("luatikztdtools@p@s@uparams", "return Vector:new{0,1,10}"),
            vparams        = get_macro_or("luatikztdtools@p@s@vparams", "return Vector:new{0,1,10}"),
            v              = token.get_macro("luatikztdtools@p@s@v"),
            curve          = token.get_macro("luatikztdtools@p@s@curve"),
            name           = token.get_macro("luatikztdtools@p@s@name"),
            partitionby    = token.get_macro("luatikztdtools@p@s@partitionby"),
            adaptive       = token.get_macro("luatikztdtools@p@s@adaptive"),
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
            arrowtip       = token.get_macro("luatikztdtools@p@c@arrowtip"),
            arrowtail      = token.get_macro("luatikztdtools@p@c@arrowtail"),
            filter         = get_macro_or("luatikztdtools@p@c@filter", "return true"),
            partitionby    = token.get_macro("luatikztdtools@p@c@partitionby"),
            adaptive       = token.get_macro("luatikztdtools@p@c@adaptive")
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
            filter         = get_macro_or("luatikztdtools@p@solid@filter", "return true"),
            partitionby    = token.get_macro("luatikztdtools@p@solid@partitionby")
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
