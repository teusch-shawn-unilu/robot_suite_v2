import { WifiPage } from "/static/wifiPage.js"
import { MainPage } from "/static/mainPage.js"

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

document.addEventListener("DOMContentLoaded", function () {
    const menu = document.getElementById("menu");

    const indexSectionBtn = document.getElementById("index-menu-btn");
    const indexPage = new MainPage();
    var currentPage = indexPage;

    currentPage.start();

    const wifiSectionBtn = document.getElementById("wifi-menu-btn");
    const wifiPage = new WifiPage();

    indexSectionBtn.addEventListener("click", function () {
        currentPage.stop();
        currentPage = indexPage;
        currentPage.start()
        menu.classList.toggle("show");
    });

    wifiSectionBtn.addEventListener("click", function () {
        currentPage.stop();
        currentPage = wifiPage;
        currentPage.start();
        menu.classList.toggle("show");
    });
});
