1、图片格式转换
	$ for i in *.bmp;	do convert ${i} ${i%bmp}jpg; done
	$ rm -rf *.bmp
2、
	$ mycount=0; while(($mycount<3)); do echo "mycount=$mycount"; gnome-screenshot --file=$mycount.png; sleep 2; ((mycount=$mycount+1)); done;

for i in *.tif; do convert ${i} ${i%tif}pgm; echo "count=$i"; done
