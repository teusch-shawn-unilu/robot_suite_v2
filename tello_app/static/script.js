show_menu = document.getElementsByClassName("show-menu");
hide_menu = document.getElementsByClassName("hide-menu");
menu = document.getElementsByClassName("menu");

show_menu.setAttribute("onclick", menu.classList.toggle());