--- Vector class
--- @class Vector
--- @field vector table<number> The 1xn vector data
local Vector = {}
Vector.__index = Vector

local Recovery = require "lua-tikz3dtools-recovery"
local Matrix
--- Matrix injection
--- @param matclass Matrix the class is its own metatable
function Vector:_set_matrix_class(matclass)
    Matrix = matclass
end

--- Create a new 1xn Vector object, without validation
--- @param vector table<number> The 1xn vector data
--- @return Vector a 1xn vector
function Vector:_new(vector)
    return setmetatable(vector, Vector)
end

--- Create a new 1xn Vector object, with validation
--- @param vector table<number> The 1xn vector data
--- @return Vector a 1xn vector
function Vector:new(vector)
    assert(
        type(vector) == "table",
        "Vector input must be a table."
    )
    local length = #vector
    assert(
        length > 0,
        "Vector must have greater than zero length."
    )
    for index = 1, length do
        assert(
            type(vector[index]) == "number",
            "Vector entries must be numbers."
        )
    end
    return Vector:_new(vector)
end


--- copy of Vector object
--- @return Vector A deep copy of self
function Vector:copy()
    local n = #self
    local t = {}
    for i = 1, n do
        t[i] = self[i]
    end
    return Vector:_new(t)
end

--- Binary verification for vector ops
--- @param other Vector hopefully a vector
function Vector:binary_verify(other)
    assert(
        getmetatable(other) == Vector,
        "Other must be a vector."
    )
    assert(#self == #other, "Other must be same size as self.")
end

--- Homogeneous addition of Vector objects, without validation
--- @param other Vector A vector of same size
--- @return Vector The sum of self and other
function Vector:_hadd(other)
    local lo = #other
    local V = {}
    for i = 1, lo - 1 do
        V[i] = self[i] + other[i]
    end
    V[lo] = self[lo]
    return Vector:_new(V)
end

--- Homogeneous addition of Vector objects, with validation
--- @param other Vector A vector of same size
--- @return Vector The sum of self and other
function Vector:hadd(other)
    self:binary_verify(other)
    return self:_hadd(other)
end

--- Addition of Vector objects, without validation
--- @param other Vector A vector of same size
--- @return Vector The sum of self and other
function Vector:_add(other)
    local lo = #other
    local V = {}
    for i = 1, lo do
        V[i] = self[i] + other[i]
    end
    return Vector:_new(V)
end

--- Addition of Vector objects, without validation
--- @param other Vector A vector of same size
--- @return Vector The sum of self and other
function Vector:add(other)
    self:binary_verify(other)
    return self:_add(other)
end

--- Homogeneous scaling of Vector objects, without validation
--- @param scalar number The scalar
--- @return Vector The scale of self by scalar
function Vector:_hscale(scalar)
    local ls = #self
    local V = {}
    for i = 1, ls - 1 do
        V[i] = self[i] * scalar
    end
    V[ls] = self[ls]
    return Vector:_new(V)
end

--- Homogeneous scaling of Vector objects, with validation
--- @param scalar number The scalar
--- @return Vector The scale of self by scalar
function Vector:hscale(scalar)
    assert(type(scalar) == "number", "Other must be a number.")
    return self:_hscale(scalar)
end

--- Scaling of Vector objects, without validation
--- @param scalar number The scalar
--- @return Vector The scale of self by scalar
function Vector:_scale(scalar)
    local ls = #self
    local V = {}
    for i = 1, ls do
        V[i] = self[i] * scalar
    end
    return Vector:_new(V)
end

--- Scaling of Vector objects
--- @param scalar number The scalar
--- @return Vector The scale of self by scalar
function Vector:scale(scalar)
    assert(type(scalar) == "number", "Other must be a number.")
    return self:_scale(scalar)
end

--- Homogeneous subtraction of Vector objects, without validation
--- @param other Vector A vector of same size
--- @return Vector The difference of self and other
function Vector:_hsub(other)
    return self:_hadd(other:_hscale(-1))
end

--- Homogeneous subtraction of Vector objects, with validation
--- @param other Vector A vector of same size
--- @return Vector The difference of self and other
function Vector:hsub(other)
    self:binary_verify(other)
    return self:_hsub(other)
end

--- Subtraction of Vector objects, without validation
--- @param other Vector A vector of same size
--- @return Vector The difference of self and other
function Vector:_sub(other)
    return self:_add(other:_scale(-1))
end

--- Subtraction of Vector objects, with validation
--- @param other Vector A vector of same size
--- @return Vector The difference of self and other
function Vector:sub(other)
    self:binary_verify(other)
    return self:_sub(other)
end

--- Homogeneous inner product of Vector objects, without validation
--- @param other Vector The RHS
--- @return number The homogeneous inner product of self with other
function Vector:_hinner(other)
    local ls = #self
    local result = 0
    for i = 1, ls - 1 do
        result = result + self[i] * other[i]
    end
    return result
end

--- Homogeneous inner product of Vector objects, with validation
--- @param other Vector The RHS
--- @return number The homogeneous inner product of self with other
function Vector:hinner(other)
    self:binary_verify(other)
    return self:_hinner(other)
end

--- Inner product of Vector objects, without validation
--- @param other Vector The RHS
--- @return number The inner product of self with other
function Vector:_inner(other)
    local ls = #self
    local result = 0
    for i = 1, ls do
        result = result + self[i] * other[i]
    end
    return result
end

--- Inner product of Vector objects, with validation
--- @param other Vector The RHS
--- @return number The inner product of self with other
function Vector:inner(other)
    self:binary_verify(other)
    return self:_inner(other)
end

--- Homogeneous inner vector product of Vector objects, without validation
--- @param other Vector The RHS
--- @return Vector The componentwise homogeneous product of self with other
function Vector:_hinnervec(other)
    local ls = #self
    local result = {}
    for i = 1, ls - 1 do
        result[i] = self[i] * other[i]
    end
    result[ls] = self[ls]
    return Vector:_new(result)
end

--- Homogeneous inner vector product of Vector objects, with validation
--- @param other Vector The RHS
--- @return Vector The componentwise homogeneous product of self with other
function Vector:hinnervec(other)
    self:binary_verify(other)
    return self:_hinnervec(other)
end

--- Homogeneous cross product, unvalidated
--- @param other Vector the RHS
--- @return Vector the product
function Vector:_hcross(other)
    return Vector:_new{
        self[2] * other[3] - self[3] * other[2],
        self[3] * other[1] - self[1] * other[3],
        self[1] * other[2] - self[2] * other[1],
        1
    }
end

--- Homogeneous cross product, validated
--- @param other Vector the RHS
--- @return Vector the product
function Vector:hcross(other)
    self:binary_verify(other)
    return self:_hcross(other)
end

--- cross product, unvalidated
--- @param other Vector the RHS
--- @return Vector the product
function Vector:_cross(other)
    return Vector:_new{
        self[2] * other[3] - self[3] * other[2],
        self[3] * other[1] - self[1] * other[3],
        self[1] * other[2] - self[2] * other[1]
    }
end

--- cross product, validated
--- @param other Vector the RHS
--- @return Vector the product
function Vector:cross(other)
    self:binary_verify(other)
    return self:_cross(other)
end

--- Homogeneous normalization of Vector objects
--- @return Vector The homogeneous normalization of self
function Vector:hnormalize()
    local ls = #self
    local len = 0
    for i = 1, ls - 1 do
        len = len + self[i] * self[i]
    end
    len = math.sqrt(len)
    local V = {}
    if not (len == len and len > 0 and len < math.huge) then
        for i = 1, ls - 1 do V[i] = 0 end
        V[ls] = self[ls]
        return Vector:_new(V)
    end
    for i = 1, ls - 1 do
        V[i] = self[i] / len
    end
    V[ls] = self[ls]
    return Vector:_new(V)
end

--- Normalization of Vector objects
--- @return Vector The normalization of self
function Vector:normalize()
    local ls = #self
    local len = 0
    for i = 1, ls do
        len = len + self[i] * self[i]
    end
    len = math.sqrt(len)
    local V = {}
    if not (len == len and len > 0 and len < math.huge) then
        for i = 1, ls do V[i] = 0 end
        return Vector:_new(V)
    end
    for i = 1, ls do
        V[i] = self[i] / len
    end
    return Vector:_new(V)
end

--- Homogeneous distance between Vector objects, unvalidated
--- @param other Vector A vector of same size
--- @return number The homogeneous distance between self and other
function Vector:_hdistance(other)
    local ls = #self
    local result = 0
    for i = 1, ls - 1 do
        result = result + (self[i] - other[i])^2
    end
    return math.sqrt(result)
end

--- Homogeneous distance between Vector objects, validated
--- @param other Vector A vector of same size
--- @return number The homogeneous distance between self and other
function Vector:hdistance(other)
    self:binary_verify(other)
    return self:_hdistance(other)
end


--- Distance between Vector objects, unvalidated
--- @param other Vector A vector of same size
--- @return number The distance between self and other
function Vector:_distance(other)
    local ls = #self
    local result = 0
    for i = 1, ls do
        result = result + (self[i] - other[i])^2
    end
    return math.sqrt(result)
end

--- Distance between Vector objects, validated
--- @param other Vector A vector of same size
--- @return number The distance between self and other
function Vector:distance(other)
    self:binary_verify(other)
    return self:_distance(other)
end

--- Homogeneous projection of Vector objects, unvalidated
--- @param other Vector A vector of same size
--- @return Vector The homogeneous projection of self onto other
function Vector:_hproject_onto(other)
    local denom = other:_hinner(other)
    if not (denom == denom and denom > 0 and denom < math.huge) then
        local out = {}
        for i = 1, #other - 1 do out[i] = 0 end
        out[#other] = other[#other] == other[#other] and other[#other] or 1
        return Vector:_new(out)
    end
    local scalar = self:_hinner(other) / denom
    return other:_hscale(scalar)
end

--- Homogeneous projection of Vector objects, validated
--- @param other Vector A vector of same size
--- @return Vector The homogeneous projection of self onto other
function Vector:hproject_onto(other)
    self:binary_verify(other)
    return self:_hproject_onto(other)
end

--- projection of Vector objects, unvalidated
--- @param other Vector A vector of same size
--- @return Vector The projection of self onto other
function Vector:_project_onto(other)
    local denom = other:_inner(other)
    if not (denom == denom and denom > 0 and denom < math.huge) then
        local out = {}
        for i = 1, #other do out[i] = 0 end
        return Vector:_new(out)
    end
    local scalar = self:_inner(other) / denom
    return other:_scale(scalar)
end

--- projection of Vector objects, validated
--- @param other Vector A vector of same size
--- @return Vector The projection of self onto other
function Vector:project_onto(other)
    self:binary_verify(other)
    return self:_project_onto(other)
end

--- Homogeneous orthogonal projection of Vector objects onto a plane, unvalidated
--- @param plane_basis table A 3-row basis (origin, u-dir, v-dir) in homogeneous coordinates
--- @return Vector The homogeneous orthogonal projection of self onto the plane
function Vector:_horthogonal_projection_onto_plane(plane_basis)
    local TO, TU, TV = table.unpack(plane_basis)
    local TW = TU:_hcross(TV)
    local TOS = self:_hsub(TO)
    if TW:_hinner(TOS) < 0 then
        TW = TW:_hscale(-1)
    end
    local proj = TOS:_hproject_onto(TW)
    return self:_hsub(proj)
end

--- Homogeneous orthogonal projection of Vector objects onto a plane, validated
--- @param plane_basis table A 3-row basis (origin, u-dir, v-dir) in homogeneous coordinates
--- @return Vector The homogeneous orthogonal projection of self onto the plane
function Vector:horthogonal_projection_onto_plane(plane_basis)
    assert(
        getmetatable(plane_basis) == Matrix, 
        "Must be a 2D affine basis."
    )
    assert(#plane_basis == 3 and #plane_basis[1] == 4, "Incorrect plane basis.")
    return self:_horthogonal_projection_onto_plane(plane_basis)
end

--- Homogeneous norm of Vector objects
--- @return number The homogeneous norm of self
function Vector:hnorm()
    local sum = 0
    for i = 1, #self - 1 do
        sum = sum + self[i] * self[i]
    end
    return math.sqrt(sum)
end

--- Norm of Vector objects
--- @return number The norm of self
function Vector:norm()
    local sum = 0
    for i = 1, #self do
        sum = sum + self[i] * self[i]
    end
    return math.sqrt(sum)
end

--- Homogeneous reciprocation of Vector objects
--- @return Vector The homogeneous reciprocation of self
function Vector:reciprocate_by_homogeneous()
    local V = {}
    local w = self[#self]
    for i = 1, #self - 1 do
        V[i] = self[i] / w
    end
    V[#self] = 1
    return Vector:_new(V)
end

--- Find an arbitrary vector orthogonal to self
--- @return Vector An arbitrary vector orthogonal to self
function Vector:horthogonal_vector()
    local norm = self:hnorm()
    if not (norm == norm and norm > 0 and norm < math.huge) then
        return Vector:_new{1, 0, 0, 1}
    end

    -- Cross with the least-aligned coordinate axis.  This is scale-independent
    -- and guarantees a well-conditioned nonzero result for every nonzero
    -- three-dimensional vector.
    local ax = math.abs(self[1])
    local ay = math.abs(self[2])
    local az = math.abs(self[3])
    local reference
    if ax <= ay and ax <= az then
        reference = Vector:_new{1, 0, 0, 1}
    elseif ay <= az then
        reference = Vector:_new{0, 1, 0, 1}
    else
        reference = Vector:_new{0, 0, 1, 1}
    end
    return self:_hcross(reference)
end

--- Create a homogeneous sphere point from spherical coordinates
--- @param v Vector the theta, phi, and radius
--- @return Vector
function Vector.sphere(v)
    local theta, phi, radius = v[1], v[2], v[3]
    local s = math.sin(phi)
    local x = radius * s * math.cos(theta)
    local y = radius * s * math.sin(theta)
    local z = radius * math.cos(phi)
    return Vector:_new{x, y, z, 1}
end

--- The stereographic projection
--- @param v Vector the 3D vector
--- @return Vector the stereographic projection
function Vector.stereographic_projection(v)
    local x, y, z = v[1], v[2], v[3]
    return Vector:_new{x/(1-z), y/(1-z), 0, 1}
end

--- The inverse stereographic projection
--- @param x number the x point
--- @param y number the y point
--- @return Vector the inverse stereographic projection
function Vector.inverse_stereographic_projection(v)
    local x, y = v[1], v[2]
    return Vector:_new{2*x/(1+x^2+y^2), 2*y/(1+x^2+y^2), (-1+x^2+y^2)/(1+x^2+y^2), 1}
end

--- Multiply Vector by Matrix (applies projective divide), unvalidated
--- @param matrix table The matrix
--- @return Vector The product of self and matrix, divided by homogeneous w
function Vector:_multiply(matrix)
    local M = Matrix:_new{self}:multiply(matrix)
    return M[1]:reciprocate_by_homogeneous()
end

--- Multiply Vector by Matrix (applies projective divide), validated
--- @param matrix table The matrix
--- @return Vector The product of self and matrix, divided by homogeneous w
function Vector:multiply(matrix)
    assert(getmetatable(matrix) == Matrix, "other must be a matrix.")
    local vn = #self 
    local vm = #matrix[1]
    assert(vn == vm, "Wrong size transform") 
    return self:_multiply(matrix)
end

return Vector
