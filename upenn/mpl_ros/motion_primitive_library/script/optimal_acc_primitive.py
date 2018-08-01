from sympy import *

dp = Symbol('dp')#dp = pf -p0

vf = Symbol('vf')
v0 = Symbol('v0')

T = Symbol('T')

A = Matrix([[-12/T**3, 6/T**2], [6/T**2, -2/T]])
b = Matrix([dp-v0*T, vf-v0])

d = simplify(A*b)

d3 = d[0]
d2 = d[1]

print d3
print d2

C = T**3/3*d3*d3 + T*d2*d2+d3*d2*T**2
C = simplify(C)

print "C = ", C
print "C' = ", diff(C, T, 1)
