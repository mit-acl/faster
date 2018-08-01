from sympy import *

dp = Symbol('dp')#dp = pf -p0

vf = Symbol('vf')
v0 = Symbol('v0')

af = Symbol('af')
a0 = Symbol('a0')

T = Symbol('T')

A = Matrix([[720, -360*T, 60*T*T],[-360*T, 168*T*T, -24*T*T*T], [60*T*T, -24*T*T*T, 3*T*T*T*T]])
b = Matrix([dp-v0*T-0.5*a0*T*T, vf-v0-a0*T, af-a0])

d = simplify(A*b/T**5)

d5 = d[0]
d4 = d[1]
d3 = d[2]

C = T**5/20*d5*d5 + T**3/3*d4*d4+T*d3*d3+T**4/4*d4*d5+T**3/3*d3*d5+T*T*d3*d4
C = simplify(C)

print "C = ", C
print "C' = ", diff(C, T, 1)
