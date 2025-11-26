import os
import io


'''

Very Scuffed 2d space character-delimited txt file to 3d comma-delimited file (i.e. works with my stuff)

'''

def convertto3d(): 
    infile = open("../TXT/start.txt", "r")
    outf = open(os.path.join("../../scenes", "Scene0"), "w")

    for line in infile:
        line = line.strip()
        
        x, y = line.split(' ')

        x = str(float(x) * 10 + 500)
        y = str(float(y) * 10 + 500)

        for i in range(5):
            outf.write(x + "," + y + "," + str(100 * i) + "\n")


convertto3d()
