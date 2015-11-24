Copy-Item -Force -Recurse  -Verbose C:\Users\Mthsci.student\Documents\robotics\2015\ftc_app\FtcRobotController\src\main\java\com\qualcomm\ftcrobotcontroller\opmodes -Destination C:\Users\Mthsci.student\Documents\robotics\2015\2015OpModes

cd C:\Users\Mthsci.student\Documents\robotics\2015\2015OpModes

$message = read-host "Please enter commit message"

git add .

git commit -m "$message"