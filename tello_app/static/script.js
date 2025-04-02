import {WifiPage} from "/static/wifiPage.js"

document.addEventListener("DOMContentLoaded", function () {
    const menu = document.getElementById("menu");
    const showMenuBtn = document.getElementById("show-menu");
    const hideMenuBtn = document.getElementById("hide-menu");
  
    showMenuBtn.addEventListener("click", function () {
        menu.classList.toggle("show");
    });

    hideMenuBtn.addEventListener("click", function () {
        menu.classList.toggle("show");
    })
  });

var test = new WifiPage()
//test.start()