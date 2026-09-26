#!/bin/bash

if [ $# -eq 0 -o $# -gt 1 ]
then
    echo "run with ./createPkg.sh teamname"
    echo "teamname can only contain lower case letters"
    exit 0
fi

pkgName=$1

str=`echo "$pkgName" | tr -d '[a-z]'`
if [ -n "$str" -o  ${#pkgName} -gt 20 ] || [ $pkgName = "test" ]
then
    echo "Invalid input!"
    echo "Only lower case letters are supportted!"
    echo "Length of teamname must be lower than 20!"
    echo "\"test\" is not allowed!"
    exit 0
fi

templatePkg="template"
echo "create package: $pkgName"
cp -r $templatePkg $pkgName
sed -i "s/$templatePkg/$pkgName/g" $pkgName/package.xml
sed -i "s/$templatePkg/$pkgName/g" $pkgName/CMakeLists.txt
sed -i "s/$templatePkg/$pkgName/g" $pkgName/src/player.cpp
sed -i "s/$templatePkg/$pkgName/g" $pkgName/src/topics.hpp
sed -i "s/$templatePkg/$pkgName/g" $pkgName/player_launch.py
sed -i "\$a$pkgName" ../teams.cfg
