#!/bin/bash

LOG_FILE="boostrap.log"

ROS_DISTROS=("iron" "humble" "foxy")

# Colors
Color_Off='\033[0m'       # Text Reset
# Bold
BRed='\033[1;31m'         # Red
BBlue='\033[1;34m'       # Green
BYellow='\033[1;33m'      # Yellow

ros_distro=""

print_info(){
    echo -e "${BBlue}[INFO] -> ${Color_Off}${1}"
}

print_warning(){
    echo -e "${BYellow}[WARN] -> ${Color_Off}${1}"
}

print_error(){
    echo -e "${BRed}[ERROR] -> ${Color_Off}${1}"
}

run_with_spinner() {
    local cmd="$1"
    local task_name="$2"

    local spinner="|/-\\"
    local pid

    # Log the output of the command and run it in the background
    $cmd >>"$LOG_FILE" 2>&1 &
    pid=$!

    local start_time=$(date +%s)
    local elapsed_time=0
    local i=0
    local last_output=""

    while kill -0 $pid 2>/dev/null; do
        # Calculate elapsed time
        elapsed_time=$(($(date +%s) - start_time))
        # Get the last line of the command output
        last_output=$(tail -n 1 "$LOG_FILE")

        i=$(((i + 1) % ${#spinner}))
        printf "\r[%c] %s | Elapsed: %d seconds | Last Output: %s" "${spinner:$i:1}" "$task_name" "$elapsed_time" "$last_output"
        sleep 0.1
    done

    printf "\r[✔] %s\n" "$task_name"
}


is_ros_distro_installed(){
    local dist_name=$1
    local setup_file="/opt/ros/${dist_name}/setup.bash"

    if [ -f "$setup_file" ]; then
        return 1
    else
        return 0
    fi
}


is_ros_installed(){
    for distro in "${ROS_DISTROS[@]}";
    do
        is_ros_distro_installed "$distro"
        if [ $? == 1 ]; then
            ros_distro="$distro"
            break
        fi
    done

    if [ "$ros_distro" == "" ]; then
        print_error "No ROS distribution was found. Please Install it first!"
        exit 1
    fi
}

ask_user_input(){
    local message=$1
    local response=""

    printf "$message"
    read -r response
    response="${response:-Y}" # default is yes
    response="${response,,}" # tolower

    if [[ $response =~ ^(yes|y)$ ]]; then
        return 1
    elif [[ $response =~ ^(no|n)$ ]]; then
        return 0
    else
        print_error "Invalid input! Please enter 'yes' or 'no'!"
        exit 1
    fi
}

install_tellopy() {
    git clone https://github.com/hanyazou/TelloPy.git tellopy
    cd tellopy
    pip install .

    cd ..
    sudo rm -rf tellopy
}


# Check if Git is installed
if [ "$(command -v git)" == "" ]; then
    ask_user_input "git is not installed. Do you want to install it? [Y/n]"
    r=$?

    if [ $r == 1 ]; then
        sudo apt install -y git
    else
        print_error "Please install git first!"
        exit 1
    fi
fi


if [ "$(command -v pip)" == "" ]; then
    ask_user_input "pip is not installed. Do you want to install it? [Y/n]"
    r=$?

    if [ $r == 1 ]; then
        sudo apt install -y python3-pip
    else
        print_error "Please install pip first!"
        exit 1
    fi
fi


is_ros_installed
source "/opt/ros/${ros_distro}/setup.sh" >> /dev/null

# Check if rosdep is installed
if [ "$(command -v rosdep)" == "" ]; then
    print_warning "rosdep is not installed. Installing it..."
    run_with_spinner "pip install rosdep" "Install rosdep"
    run_with_spinner "sudo rosdep init" "Init update"
    run_with_spinner "rosdep update" "Updating rosdep"
else
    run_with_spinner "rosdep update" "Updating rosdep"
fi

run_with_spinner "sudo apt install ros-humble-cv-bridge" "Install cv-bridge"

# run_with_spinner "rosdep install --from-paths src --ignore-src -y" "Install packages dependencies with rosdep"

run_with_spinner "pip install -r requirements.txt" "Install python libraries from requirements.txt"

run_with_spinner install_tellopy "Installing tellopy from source"


rm $LOG_FILE
