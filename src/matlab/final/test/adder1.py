#! user/bin/env python

a = int(input('Enter a: '))
b = int(input('Enter b: '))
c = int(input('Enter c: '))

sum = ((a^b)^c)

car = (a*b|b*c|c*a)

print ('sum: ', sum)

print ('car: ',car)


