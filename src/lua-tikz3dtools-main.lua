local Vector = require "lua-tikz3dtools-vector"
local Matrix = require "lua-tikz3dtools-matrix"
local Geometry = require "lua-tikz3dtools-geometry"
local Streamline = require "lua-tikz3dtools-streamline"
local Scene = require "lua-tikz3dtools-scene"

-- These classes are intentionally public to Lua expressions supplied through
-- the TeX interface.  Publish them explicitly rather than creating accidental
-- globals through bare assignments above.
_G.Vector = Vector
_G.Matrix = Matrix
_G.Geometry = Geometry
_G.Streamline = Streamline
_G.Scene = Scene

Vector:_set_matrix_class(Matrix)
Matrix:_set_vector_class(Vector)
Geometry:_set_classes(Vector, Matrix)
Scene:_set_classes(Vector, Matrix, Geometry)

Scene.register_commands()
