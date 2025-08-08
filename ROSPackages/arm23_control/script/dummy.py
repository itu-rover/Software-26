from numpy import interp

gear1 = (40./12.)*(60./12.)
gear2 = (41./1.)
print(interp(3.14/6, [-1.8, 1.8], [0, 9999]))
#print(interp(1.0, [-2.5, 2,5], [9999, 0]))
#print(interp(0.03534078598022461, [-1.8, 1.8], [0,9999]))
# map(abs_pos, (-1.8/pi*180*gear1, 1.8/pi*180*gear1) -> 0, 9999) 
# -2.5/pi*180*gear2, 2.5/pi*180*gear2 -> 0, 9999
# -2.2/pi*180*gear3, 2.2/pi*180*gear3 -> 0, 9999
# -3.14/pi*180*gear4, 3.14/pi*180*gear4 -> 0, 9999
# -3.2/pi*180*gear6, 3.2/pi*180*gear5 -> 0, 9999
# -6.28/pi*180*gear6, 6.28/pi*180*gear6 -> 0, 9999
