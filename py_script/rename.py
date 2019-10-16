# --** coding="UTF-8" **--   
import os
import re
import sys
fileList = os.listdir(r"/media/jiang/3E2053CE20538BB1/20190918_IMB_AP2Tuguan/20190918090603/RefLaser")
print("old name: "+str(fileList)[1])
currentpath = os.getcwd()
os.chdir(r"/media/jiang/3E2053CE20538BB1/20190918_IMB_AP2Tuguan/20190918090603/RefLaser")
num = 1
for fileName in fileList:
    pat = ".+\.(las)"
    pattern = re.findall(pat, fileName)
    os.rename(fileName, (str(num) + '_No.1-01-001_Lidar_HESAIGT40_TuGuan1_1.' + pattern[0])) #
    num = num + 1
print("***************************************")
os.chdir(currentpath)
sys.stdin.flush()
#print("new name: "+str(os.listdir(r"./las"))[1])
