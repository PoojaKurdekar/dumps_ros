#!/usr/bin/env python

import rospy

def AND(A, B):
	return A & B
	
def OR(A, B):
	return A | B

def NOT(A):
	return ~A+2

def NAND(A, B):
	return NOT(AND(A, B))

def NOR(A, B):
	return NOT(OR(A, B))

def XOR(A, B):
	return A ^ B

def XNOR(A, B):
	return NOT(XOR(A, B))

def HA(A, B):
	sum = XOR(A, B)
	car = AND(A, B)
	return sum, car

def FA(A, B, C):
	s = XOR(XOR(A, B), C)
	r = (A & B) | (B & C) | (C & A)
	return s, r

A = input('enter a= ')
B = input('enter b= ')
C = input('enter c= ')

X = FA(A, B, C)

print('sum: ', X[0] )
print('car: ', X[1] )



