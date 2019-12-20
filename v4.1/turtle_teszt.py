import time
import turtle
t = turtle.Pen()
t.forward(100)
t.setheading(120)
t.forward(100)
t.getscreen().getcanvas().postscript(file='outputname2.ps')
