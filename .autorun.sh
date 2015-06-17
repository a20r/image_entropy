
(cd $CATKIN_WS; catkin_make; cd -;)

read -p "Commit message: " msg
git add -A .
git commit -m "$msg"
git push
