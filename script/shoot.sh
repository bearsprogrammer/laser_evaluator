#!/bin/bash
# Red is 1
# Green is 2
# Reset is sgr0
cd ../
WS_PATH=$PWD
cd src/ 
echo -e "Current workspace location is ${WS_PATH}"
ls
echo -e "Please input node(package) name what you want for ROS system: \c "
read node 

tput setaf 2
echo -e "\v                                                                               "
  echo -e "           *                                                                   "
  echo -e "      *         *                                                              "
  echo -e "        *  *  *                    +                                           "
  echo -e "    *    * @ *    *               +++                                          "
  echo -e "        *  *  *                  +++++              ~~  ~~                     "
  echo -e "     *           *              +++ +++             ~~  ~~                     "
  echo -e "           *                   +++   +++            ~~  ~~                     "
  echo -e "                              +++     +++           ~~  ~~                     "
  echo -e "                             +++       +++          ~~  ~~                     "
  echo -e "                            +++++++++++++++         ~~  ~~                     "
  echo -e "           *               +++           +++        ~~  ~~                     "
  echo -e "                          +++             +++       ~~  ~~    ~~~    ~~  ~~~   "
  echo -e "                         +++               +++      ~~  ~~  ~~~ ~~~  ~~ ~~~~   "
  echo -e "                        +++                 +++     ~~  ~~  ~~~~~~~  ~~ ~ ~~   "
  echo -e "                       +++                   +++    ~~  ~~  ~~~      ~~~  ~~   "
  echo -e "                      +++                     +++   ~~  ~~   ~~~~~   ~~~  ~~   "
  echo -e "                                                                               "
  echo -e "    +-                          +             +--         --    +--------      "
  echo -e "    +-                         +--            +- -        --    +-       --    "
  echo -e "    +-                        +- --           +-  -       --    +-        --   "
  echo -e "    +-                       +-   --          +-   -      --    +-         --  "
  echo -e "    +-                      +-     --         +-    -     --    +-         --  "
  echo -e "    +-                     +----------        +-     -    --    +-         --  "
  echo -e "    +-                    +-         --       +-      -   --    +-         --  "
  echo -e "    +-                   +-           --      +-       -  --    +-        --   "
  echo -e "    +-                  +-             --     +-        - --    +-       --    "
  echo -e "    +--------------    +-               --    +-         ---    +--------      "
  echo -e "                                                                               "
  echo -e "                                                                                \v"
tput sgr0

catkin_create_pkg ${node} std_msgs roscpp
source ${WS_PATH}/devel/setup.bash
rospack depends1 ${node}
cd ${WS_PATH}/src/${node}/src/
cp ${WS_PATH}/script/temp/templete.cpp ./
cp ${WS_PATH}/script/temp/CMakeLists.txt ../
cp ${WS_PATH}/script/temp/package.xml ../
mv templete.cpp main.cpp
cd ${WS_PATH}
tput setaf 1
echo "======================="
echo "${node} package is created."
echo "======================="
tput sgr0
source devel/setup.bash

