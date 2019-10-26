#!/usr/bin/env Python
# coding=utf-8

import os
out = open('AccurResult_tf.txt','w')
for line in open("AccurResult-1.txt"):   
    print line
    #str = line
    list = line.split()
    print list
    str = ' '.join(list)
    out.writelines(str)
    out.writelines('\n')
out.close()



