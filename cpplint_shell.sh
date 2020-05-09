#! /bin/bash 
echo "^@^cpplint code style check through shell====^"
index=0
config=""
pwd_path=`pwd`
cpplint_path="pwd_path/cpplint.py"
echo cpplint_path=$cpplint_path

src_path="$pwd_path/src"
echo src_path=$src_path 
for file in `find $src_path -name "*.h" -type f | grep -E "\.h$|\.cc$|\.cu$|\.cpp$"`
do 
    echo file=$file 
    echo -e "\033[36m ===> [FILE] \033[0m \033[47;31m $file \033[0m"
    check_files[$index]=$file
    index=$(($index+1))
done
check_cmd=""
for file in ${check_files[*]}
do
    check_cmd="python2 $cpplint_path --linelength=80"
    echo -e "\033[33m =========== check file $file =============\033[0m"
    check_cmd="$check_cmd"$file 
    eval $check_cmd
    echo -e "\033[45m ==========================================\033[0m"    
done
