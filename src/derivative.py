from sympy import Derivative, symbols

x = symbols('x')
fx = 4 * x ** 4 + 3 * x ** 3 +  2 * x ** 2 + x + 1

fprime = Derivative(fx, x).doit() 
print("fx 의 도함수는 : ", fprime, "입니다")
n = fprime.subs({x: 3})
print("fx에서 x = 3 에서의 순간변화율(미분계수는) ", n , "입니다")