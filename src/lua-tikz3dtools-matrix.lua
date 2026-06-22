--- Matrix class
--- @class Matrix
--- @field matrix table<Vector>
local Matrix = {}
Matrix.__index = Matrix

local Vector = nil
--- Vector injection
--- @param vclass Vector the class is its own metatable
function Matrix:_set_vector_class(vclass)
    Vector = vclass
end

--- Matrix constructor, unverified
--- @param matrix table<Vector>
--- @return Matrix
function Matrix:_new(matrix)
    return setmetatable(matrix, Matrix)
end

--- Matrix constructor, verified
--- @param matrix table<Vector>
--- @return Matrix
function Matrix:new(matrix)
    assert(
        type(matrix) == "table",
        "matrix shell must be a table"
    )
    local rows = #matrix 
    assert(rows > 0, "matrix must not be empty")
    local cols = #matrix[1]
    for row = 1, rows do 
        assert(
            getmetatable(matrix[row]) == Vector,
            "Matrix row must be vector"
        )
        if row > 1 then 
            assert(
                #matrix[row] == cols,
                "Matrix is not rectangular"
            )
        end
    end
    return Matrix:_new(matrix)
end

--- Deep copy of Matrix object
--- @return Matrix A deep copy of self
function Matrix:copy()
    local m = {}
    for i = 1, #self do 
        m[i] = self[i]:copy()
    end 
    return Matrix:_new(m)
end

--- swap two columns of the Matrix
--- @param c1 number The first column
--- @param c2 number The second column
function Matrix:swap_columns(c1, c2)
    for i = 1, #self do
        local t = self[i][c1]
        self[i][c1] = self[i][c2]
        self[i][c2] = t
    end
end

--- add a scalar multiple of one column to another column
--- @param src_col number The source column
--- @param scalar number The scalar multiple
--- @param dest_col number The destination column
function Matrix:add_scalar_multiple_of_column(src_col, scalar, dest_col)
    local l = #self
    for i = 1, l do
        self[i][dest_col] = self[i][dest_col] + scalar * self[i][src_col]
    end
end

--- scale a column by a scalar
--- @param col number The column
--- @param scalar number The scalar
function Matrix:scale_column(col, scalar)
    local l = #self
    for i = 1, l do
        self[i][col] = self[i][col] * scalar
    end
end

--- Elementwise addition of Matrix objects, unverified
--- @param other Matrix A matrix of the same shape
--- @return Matrix The elementwise sum of self and other
function Matrix:_add(other)
    local result = {}
    for i = 1, #self do 
        result[i] = self[i]:_add(other[i])
    end
    return Matrix:_new(result)
end

--- Binary matrix verification
--- @param other Matrix hopefully a matrix
function Matrix:binary_verify(other)
    assert(
        getmetatable(other) == Matrix,
        "Other must be a Matrix."
    )
    assert(
        #self == #other and #self[1] == #other[1], 
        "Other must be same size as self."
    )
end

--- Elementwise addition of Matrix objects, verified
--- @param other Matrix A matrix of the same shape
--- @return Matrix The elementwise sum of self and other
function Matrix:add(other)
    self:binary_verify(other)
    return self:_add(other)
end

--- Elementwise subtraction of Matrix objects, unverified
--- @param other Matrix A matrix of the same shape
--- @return Matrix The elementwise difference of self and other
function Matrix:_sub(other)
    local result = {}
    for i = 1, #self do 
        result[i] = self[i]:_sub(other[i])
    end
    return Matrix:_new(result)
end

--- Elementwise subtraction of Matrix objects, verified
--- @param other Matrix A matrix of the same shape
--- @return Matrix The elementwise difference of self and other
function Matrix:sub(other)
    self:binary_verify(other)
    return self:_sub(other)
end

--- Elementwise scale of Matrix objects, unverified
--- @param other Matrix A matrix of the same shape
--- @return Matrix The elementwise scale of self and other
function Matrix:_scale(other)
    local result = {}
    for i = 1, #self do 
        result[i] = self[i]:_scale(other)
    end
    return Matrix:_new(result)
end

--- Elementwise scale of Matrix objects, verified
--- @param other Matrix A matrix of the same shape
--- @return Matrix The elementwise scale of self and other
function Matrix:scale(other)
    assert(type(other) == "number", "other must be a scalar")
    return self:_scale(other)
end

--- Column reduction of the Matrix to RREF form
--- THIS IS AI GENERATED!!! As of yet, I am still learning linear algebra.
--- @return Vector|nil The solution vector (last row of RREF)
function Matrix:column_reduction()
    local cols = self:copy()
    local numcols = #cols
    if numcols == 0 then return nil end
    local numrows = #cols[1]
    local vars = numcols - 1
    if vars < 1 then return nil end
    local eps = 1e-12

    local rank = 0
    local row = 1
    local pivot_cols = {}

    for col = 1, vars do
        if row > numrows then break end

        local pivot_row = nil
        local maxval = eps
        for r = row, numrows do
            local value = math.abs(cols[col][r])
            if value > maxval then
                maxval = value
                pivot_row = r
            end
        end

        if pivot_row then
            if pivot_row ~= row then
                for c = 1, numcols do
                    cols[c][row], cols[c][pivot_row] = cols[c][pivot_row], cols[c][row]
                end
            end

            local pivot = cols[col][row]
            for c = 1, numcols do
                cols[c][row] = cols[c][row] / pivot
            end

            for r = 1, numrows do
                if r ~= row then
                    local factor = cols[col][r]
                    if math.abs(factor) > eps then
                        for c = 1, numcols do
                            cols[c][r] = cols[c][r] - factor * cols[c][row]
                        end
                        cols[col][r] = 0
                    end
                end
            end

            pivot_cols[#pivot_cols + 1] = col
            rank = rank + 1
            row = row + 1
        end
    end

    for r = rank + 1, numrows do
        local all_zero = true
        for c = 1, vars do
            if math.abs(cols[c][r]) > eps then
                all_zero = false
                break
            end
        end
        if all_zero and math.abs(cols[numcols][r]) > eps then
            return nil
        end
    end

    local sol = {}
    for i = 1, vars do
        sol[i] = 0
    end
    for k, pcol in ipairs(pivot_cols) do
        sol[pcol] = cols[numcols][k]
    end

    for _, v in ipairs(sol) do
        if v ~= v then return nil end
    end

    return Vector:_new(sol)
end

--- Transpose of Matrix object
--- @return Matrix The transposed matrix
function Matrix:transpose()
    local numrows = #self
    local numcols = #self[1]
    local transposed = {}
    for j = 1, numcols do
        transposed[j] = {}
        for i = 1, numrows do
            transposed[j][i] = self[i][j]
        end
        transposed[j] = Vector:_new(transposed[j])
    end
    return Matrix:_new(transposed)
end


--- Inverse of the Matrix (row-vector convention, column GJ)
--- THIS IS AI GENERATED AS OF YET, SORRY.
--- @return Matrix|nil The inverse matrix, or nil if not invertible
function Matrix:inverse()
    local n = #self
    assert(n == #self[1], "Matrix must be square.")

    local eps = 1e-12

    local A = self:copy()

    local I_table = {}
    for i = 1, n do
        I_table[i] = {}
        for j = 1, n do
            I_table[i][j] = (i == j) and 1 or 0
        end
        I_table[i] = Vector:_new(I_table[i])
    end
    local I = Matrix:_new(I_table)

    for pivot = 1, n do
        local pivot_row = pivot
        local max_abs = math.abs(A[pivot][pivot])

        for r = pivot + 1, n do
            local v = math.abs(A[r][pivot])
            if v > max_abs then
                max_abs = v
                pivot_row = r
            end
        end

        if max_abs <= eps then
            return nil
        end

        if pivot_row ~= pivot then
            A[pivot], A[pivot_row] = A[pivot_row], A[pivot]
            I[pivot], I[pivot_row] = I[pivot_row], I[pivot]
        end

        local pivot_value = A[pivot][pivot]
        for c = 1, n do
            A[pivot][c] = A[pivot][c] / pivot_value
            I[pivot][c] = I[pivot][c] / pivot_value
        end

        for r = 1, n do
            if r ~= pivot then
                local factor = A[r][pivot]
                if math.abs(factor) > eps then
                    for c = 1, n do
                        A[r][c] = A[r][c] - factor * A[pivot][c]
                        I[r][c] = I[r][c] - factor * I[pivot][c]
                    end
                    A[r][pivot] = 0
                end
            end
        end
    end

    return I
end

--- Convert to a basis from a simplex
--- @return Matrix The basis matrix
function Matrix:hto_basis()
    local numrows = #self
    local numcols = #self[1]
    local copy = self:copy()
    for i = 1, numrows do
        if i ~= 1 then
            for j = 1, numcols - 1 do
                copy[i][j] = copy[i][j] - copy[1][j]
            end
        end
    end
    return copy
end

--- Homogeneous reciprocation of Matrix objects
--- @return Matrix The homogeneous reciprocation of self
function Matrix:reciprocate_by_homogeneous()
    local result = {}
    for i = 1, #self do
        result[i] = {}
        local w = self[i][#self[i]]
        for j = 1, #self[i] - 1 do
            result[i][j] = self[i][j] / w
        end
        result[i][#self[i]] = 1
        result[i] = Vector:_new(result[i])
    end
    return Matrix:_new(result)
end

--- Multiply two Matrix objects, unverified
--- @param other Matrix|Vector The RHS
--- @return Matrix|Vector The product of self and other
--- @param reciprocate boolean|nil If true, apply homogeneous reciprocation for non-4x4 results (default true for non-flag)
function Matrix:_multiply(other)
    local Arows = #self
    local Acols = #self[1]
    local Bcols = #other[1]
    local product = {}
    for row = 1, Arows do
        product[row] = {}
        for col = 1, Bcols do
            product[row][col] = 0
            for k = 1, Acols do
                product[row][col] = product[row][col] + self[row][k] * other[k][col]
            end
        end
        product[row] = Vector:_new(product[row])
    end
    return Matrix:_new(product)
end

--- Multiply two Matrix objects, veri....
--- @param other Matrix|Vector The RHS
--- @return Matrix|Vector The product of self and other
--- @param reciprocate boolean|nil If true, apply homogeneous reciprocation for non-4x4 results (default true for non-flag)
function Matrix:multiply(other)
    local Arows = #self
    local Acols = #self[1]
    local Bcols = #other[1]
    local product = {}
    for row = 1, Arows do
        product[row] = {}
        for col = 1, Bcols do
            product[row][col] = 0
            for k = 1, Acols do
                product[row][col] = product[row][col] + self[row][k] * other[k][col]
            end
        end
        product[row] = Vector:_new(product[row])
    end
    return Matrix:_new(product)
end

--- Get 3D bounding box of Matrix points
--- @return table A table with 'min' and 'max' Vector objects
function Matrix:get_bbox3()
    local min_x, min_y, min_z = math.huge, math.huge, math.huge
    local max_x, max_y, max_z = -math.huge, -math.huge, -math.huge
    for _, row in ipairs(self) do
        if row[1] < min_x then min_x = row[1] end
        if row[2] < min_y then min_y = row[2] end
        if row[3] < min_z then min_z = row[3] end
        if row[1] > max_x then max_x = row[1] end
        if row[2] > max_y then max_y = row[2] end
        if row[3] > max_z then max_z = row[3] end
    end
    return {
        min = Vector:_new{min_x, min_y, min_z, 1},
        max = Vector:_new{max_x, max_y, max_z, 1}
    }
end

--- Get 2D bounding box of Matrix points
--- @return table A table with 'min' and 'max' Vector objects
function Matrix:get_bbox2()
    local min_x, min_y = math.huge, math.huge
    local max_x, max_y = -math.huge, -math.huge
    for _, row in ipairs(self) do
        if row[1] < min_x then min_x = row[1] end
        if row[2] < min_y then min_y = row[2] end
        if row[1] > max_x then max_x = row[1] end
        if row[2] > max_y then max_y = row[2] end
    end
    return {
        min = Vector:_new{min_x, min_y, 0, 1},
        max = Vector:_new{max_x, max_y, 0, 1}
    }
end

--- Check if two 3D bounding boxes overlap (using pre-computed bboxes when available)
--- @param other Matrix The other Matrix
--- @param a_bbox table|nil Pre-computed bbox for self
--- @param b_bbox table|nil Pre-computed bbox for other
--- @return boolean True if they overlap, false otherwise
function Matrix:bboxes_overlap3(other, a_bbox, b_bbox)
    a_bbox = a_bbox or self:get_bbox3()
    b_bbox = b_bbox or other:get_bbox3()
    return not (
        a_bbox.max[1] < b_bbox.min[1]
        or a_bbox.min[1] > b_bbox.max[1]
        or a_bbox.max[2] < b_bbox.min[2]
        or a_bbox.min[2] > b_bbox.max[2]
        or a_bbox.max[3] < b_bbox.min[3]
        or a_bbox.min[3] > b_bbox.max[3]
    )
end

--- Check if two 2D bounding boxes overlap (using pre-computed bboxes when available)
--- @param other Matrix The other Matrix
--- @param a_bbox table|nil Pre-computed bbox for self
--- @param b_bbox table|nil Pre-computed bbox for other
--- @return boolean True if they overlap, false otherwise
function Matrix:bboxes_overlap2(other, a_bbox, b_bbox)
    a_bbox = a_bbox or self:get_bbox2()
    b_bbox = b_bbox or other:get_bbox2()
    return not (
        a_bbox.max[1] < b_bbox.min[1]
        or a_bbox.min[1] > b_bbox.max[1]
        or a_bbox.max[2] < b_bbox.min[2]
        or a_bbox.min[2] > b_bbox.max[2]
    )
end

--- Compute the homogeneous centroid of the Matrix points
--- @return Vector The homogeneous centroid
function Matrix:hcentroid()
    local ns = #self
    local dim = #self[1]
    local centroid = self[1]
    for i = 2, ns do
        centroid = centroid:_hadd(self[i])
    end
    centroid = centroid:_hscale(1 / ns)
    return centroid
end


--- Sort points in Matrix by angle around their homogeneous centroid
--- AI GENERATED FOR SPEED.
--- @return table The sorted points (as plain table, not Matrix)
function Matrix:hcentroid_sort()
    local num = #self
    local centroid = self:hcentroid()
    local P = Vector:_new(self[1])
    local u = Vector:_new(self[2]):_hsub(P):hnormalize()
    local v = Vector:_new(self[3]):_hsub(P)
    local normal = u:_hcross(v):hnormalize()
    v = normal:_hcross(u):hnormalize()
    local angles = {}
    for i, p in ipairs(self) do
        local rel = Vector:_new(p):_hsub(centroid)
        local x = rel:_hinner(u)
        local y = rel:_hinner(v)
        local angle = math.atan2(y, x)
        angles[i] = {angle = angle, index = i}
    end
    table.sort(angles, function(a, b) return a.angle < b.angle end)
    local sorted = {}
    for i, a in ipairs(angles) do
        sorted[i] = self[a.index]
    end
    return sorted
end

--- Check if all elements in the Matrix are numeric
--- @return boolean True if all elements are numeric, false otherwise
function Matrix:is_numeric()
    for _, row in ipairs(self) do
        for _, val in ipairs(row) do
            if type(val) ~= "number" then
                return false
            end
        end
    end
    return true
end

--- 3D rotation about an arbitrary axis through the origin
--- @param axis Vector The axis direction
--- @param theta number The rotation angle in radians
--- @return Matrix The rotation matrix
function Matrix.axis_angle(axis, theta)
    assert(getmetatable(axis) == Vector, "axis must be a Vector.")
    axis = axis:hnormalize()
    local x = axis[1]
    local y = axis[2]
    local z = axis[3]
    local c = math.cos(theta)
    local s = math.sin(theta)
    local t = 1 - c
    return Matrix:_new{
        Vector:_new{t * x * x + c,     t * x * y + s * z, t * x * z - s * y, 0},
        Vector:_new{t * x * y - s * z, t * y * y + c,     t * y * z + s * x, 0},
        Vector:_new{t * x * z + s * y, t * y * z - s * x, t * z * z + c,     0},
        Vector:_new{0, 0, 0, 1}
    }
end

--- 3D shear matrix
--- @param kxy number|nil x shear from y
--- @param kxz number|nil x shear from z
--- @param kyx number|nil y shear from x
--- @param kyz number|nil y shear from z
--- @param kzx number|nil z shear from x
--- @param kzy number|nil z shear from y
--- @return Matrix The shear matrix
function Matrix.shear(kxy, kxz, kyx, kyz, kzx, kzy)
    kxy = kxy or 0
    kxz = kxz or 0
    kyx = kyx or 0
    kyz = kyz or 0
    kzx = kzx or 0
    kzy = kzy or 0
    return Matrix:_new{
        Vector:_new{1,   kyx, kzx, 0},
        Vector:_new{kxy, 1,   kzy, 0},
        Vector:_new{kxz, kyz, 1,   0},
        Vector:_new{0,   0,   0,   1}
    }
end

--- 3D scale about an arbitrary axis through the origin
--- @param axis Vector The axis direction (nonzero)
--- @param scale number The scale factor along the axis
--- @return Matrix The 4x4 homogeneous scaling matrix
function Matrix.scale_axis(axis, scale)
    assert(getmetatable(axis) == Vector, "axis must be a Vector.")
    assert(type(scale) == "number", "scale must be a number.")
    -- normalize axis to unit length (use your Vector API's normalize method)
    local u = axis:hnormalize()  -- replace with :normalize() if your API uses that
    local x = u[1]
    local y = u[2]
    local z = u[3]
    local k = scale - 1

    return Matrix:_new{
        Vector:_new{1 + k * x * x, k * x * y,     k * x * z,     0},
        Vector:_new{k * x * y,     1 + k * y * y, k * y * z,     0},
        Vector:_new{k * x * z,     k * y * z,     1 + k * z * z, 0},
        Vector:_new{0, 0, 0, 1}
    }
end

--- 3D translation matrix
--- @param delta Vector The translation vector
--- @return Matrix The translation matrix
function Matrix.translate(delta)
    assert(getmetatable(delta) == Vector, "delta must be a Vector.")
    return Matrix:_new{
        Vector:_new{1, 0, 0, 0},
        Vector:_new{0, 1, 0, 0},
        Vector:_new{0, 0, 1, 0},
        Vector:_new{delta[1], delta[2], delta[3], 1}
    }
end


--- 3D ZYZ Euler rotation matrix
--- @param angles Vector The Euler angles (alpha, beta, gamma)
--- @return Matrix The rotation matrix
function Matrix.zyzrotation(angles)
    assert(getmetatable(angles) == Vector, "angles must be a Vector.")
    return Matrix.axis_angle(Vector:_new{0, 0, 1, 1}, angles[1])
        :multiply(Matrix.axis_angle(Vector:_new{0, 1, 0, 1}, angles[2]))
        :multiply(Matrix.axis_angle(Vector:_new{0, 0, 1, 1}, angles[3]))
end

--- identity matrix
--- @return Matrix The identity matrix
function Matrix.identity()
    return Matrix:_new{
        Vector:_new{1, 0, 0, 0},
        Vector:_new{0, 1, 0, 0},
        Vector:_new{0, 0, 1, 0},
        Vector:_new{0, 0, 0, 1}
    }
end

--- perspective along an arbitrary axis
--- @param axis Vector The perspective axis and strength
--- @return Matrix The perspective matrix
function Matrix.perspective(axis)
    assert(getmetatable(axis) == Vector, "axis must be a Vector.")
    return Matrix:_new{
        Vector:_new{1, 0, 0, axis[1]},
        Vector:_new{0, 1, 0, axis[2]},
        Vector:_new{0, 0, 1, axis[3]},
        Vector:_new{0, 0, 0, 1}
    }
end

--- apply a transformation about a point
--- @param point Vector The fixed point
--- @param transformation Matrix The transformation to apply
--- @return Matrix The composed transformation
function Matrix.transform_about(point, transformation)
    assert(getmetatable(point) == Vector, "point must be a Vector.")
    return Matrix.translate(Vector:_new{-point[1], -point[2], -point[3], 1})
        :multiply(transformation)
        :multiply(Matrix.translate(point))
end

return Matrix