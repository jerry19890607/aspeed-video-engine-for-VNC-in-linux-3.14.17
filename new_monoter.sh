case $1 in
0)
while [ 1 ] ;do for i in {1..7};do ./moniter.sh $i ;sleep 20;done;done
;;
1)
xrandr --output VGA-1 --mode 800x600
;;
2)
xrandr --output VGA-1 --mode 1024x768
;;
3)
xrandr --output VGA-1 --mode 640x480
;;
4)
xrandr --output VGA-1 --mode 1920x1080
;;
5)
xrandr --output VGA-1 --mode 1440x900
;;
6)
xrandr --output VGA-1 --mode 1680x1050
;;
7)
xrandr --output VGA-1 --mode 1280x1024
;;
esac