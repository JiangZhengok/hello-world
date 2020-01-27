1、图片格式转换
	$ for i in *.bmp;	do convert ${i} ${i%bmp}jpg; done
	$ rm -rf *.bmp
2、每间隔固定时间执行命令（例如截图）
	$ mycount=0; while(($mycount<3)); do echo "mycount=$mycount"; gnome-screenshot --file=$mycount.png; sleep 2; ((mycount=$mycount+1)); done;


