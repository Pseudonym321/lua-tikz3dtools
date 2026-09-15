--- Streamline helpers
---
--- Numerical integral curves are represented as ordinary callable parametric
--- maps on [0,1].  Numerical failures are conservative termination conditions:
--- a bad field sample truncates the streamline at the last valid point rather
--- than aborting the surrounding TeX job.
local Streamline = {}

local Vector = require "lua-tikz3dtools-vector"
local Recovery = require "lua-tikz3dtools-recovery"

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

local function safe_start(start)
    if valid_point(start) then return start:copy() end
    Recovery.log_once("streamline-start-fallback", "streamline recovered", "invalid start point; using the origin")
    return Vector:new{0, 0, 0, 1}
end

local function evaluate_field(field, point)
    local ok, value = pcall(field, point[1], point[2], point[3])
    if not ok or not valid_point(value) then
        Recovery.log_once(
            "streamline-field",
            "streamline truncated",
            ok and "nonfinite field sample" or "field evaluation failed"
        )
        return nil
    end
    return value
end

local function rk4_step(field, point, step)
    local k1 = evaluate_field(field, point)
    if not k1 then return nil end
    local k2 = evaluate_field(field, point:hadd(k1:hscale(step / 2)))
    if not k2 then return nil end
    local k3 = evaluate_field(field, point:hadd(k2:hscale(step / 2)))
    if not k3 then return nil end
    local k4 = evaluate_field(field, point:hadd(k3:hscale(step)))
    if not k4 then return nil end

    local delta = k1:hscale(1 / 6)
        :hadd(k2:hscale(1 / 3))
        :hadd(k3:hscale(1 / 3))
        :hadd(k4:hscale(1 / 6))
        :hscale(step)
    local candidate = point:hadd(delta)
    if not valid_point(candidate) then
        Recovery.log_once(
            "streamline-step-nonfinite",
            "streamline truncated",
            "RK4 produced a non-finite point"
        )
        return nil
    end
    return candidate
end

local function linear_sample_map(points)
    local intervals = #points - 1

    return function(t)
        if not finite_number(t) then
            return points[1]:copy()
        end
        if t < 0 or t > 1 then
            t = math.max(0, math.min(1, t))
        end
        if intervals <= 0 or t == 0 then return points[1]:copy() end
        if t == 1 then return points[#points]:copy() end

        local scaled = t * intervals
        local index = math.min(math.floor(scaled), intervals - 1)
        local fraction = scaled - index
        local a = points[index + 1]
        local b = points[index + 2]
        return a:hadd(b:hsub(a):hscale(fraction))
    end
end

--- Integrate a 3D vector field with classical fourth-order Runge--Kutta.
--- Invalid samples terminate integration at the last good point.  Invalid
--- setup returns a constant map instead of raising an error.
--- @param field function number,number,number -> Vector
--- @param start Vector homogeneous 3D starting point
--- @param step number RK4 step size
--- @param steps integer maximum number of RK4 steps
--- @param stop function|nil optional predicate stop(point) -> boolean
--- @return function alpha unit-interval sampled map
--- @return integer intervals number of retained RK4 intervals
function Streamline.rk4(field, start, step, steps, stop)
    local first = safe_start(start)
    local points = {first}

    if type(field) ~= "function" then
        Recovery.log_once("streamline-field-fallback", "streamline recovered", "field was not callable; using a constant map")
        return linear_sample_map(points), 0
    end
    if not (finite_number(step) and step ~= 0) then
        Recovery.log_once("streamline-step-fallback", "streamline recovered", "step was not a nonzero finite number; using a constant map")
        return linear_sample_map(points), 0
    end
    if not (type(steps) == "number" and steps >= 1 and steps % 1 == 0) then
        Recovery.log_once("streamline-steps-fallback", "streamline recovered", "steps was not a positive integer; using a constant map")
        return linear_sample_map(points), 0
    end
    if stop ~= nil and type(stop) ~= "function" then
        stop = nil
    end

    for _ = 1, steps do
        local current = points[#points]
        local candidate = rk4_step(field, current, step)
        if not candidate then break end

        if stop ~= nil then
            local ok, should_stop = pcall(stop, candidate)
            if not ok then
                Recovery.log_once(
                    "streamline-stop-error",
                    "streamline stop predicate ignored",
                    should_stop
                )
            elseif should_stop then
                break
            end
        end
        points[#points + 1] = candidate
    end

    return linear_sample_map(points), #points - 1
end

return Streamline
