export class WifiPage {
    constructor() {
        this.wifiInterfaces = document.getElementById("wifi-networks");
        this.networks = [];
    }

    start() {
        console.log("starting wifi page")
        this.intervalId = window.setInterval(async () => {
            try {
                var response = await fetch("/get_wifi_networks");

                if (!response.ok) 
                    console.log("Issue fetching wifi networks");

                this.networks = await response.json();
                this.networks = this.networks["networks"];

                response = await fetch("/get_connected_network");

                if (!response.ok)
                    console.log("Issue fetching connected network");

                this.connected_network = await response.json();

                this.updateWifiInterface()

            } catch (error) {
                console.log("Error fetching wifi networks " + error);
            }
        }, 10000);
    }

    stop() {
        clearInterval(this.intervalId);
        this.intervalId = null;
        this.wifiInterfaces.textContent = '';
        console.log("stoping wifi page")
    }

    updateWifiInterface() {
        this.wifiInterfaces.textContent = '';
        
        this.networks.forEach(element => {
            this.wifiInterfaces.appendChild(
                this.getWifiTemplate(element[0], element[1])
            );
        })
    }

    getWifiTemplate(networkName, hasPassword) {
        const wifiName = document.createElement("p");
        wifiName.innerHTML = networkName;

        const isSecure = document.createElement("p");
        isSecure.innerHTML = hasPassword ? "pw required": "no pw";

        const wifiInterface = document.createElement("div");
        wifiInterface.classList.add("wifi-network");
        wifiInterface.appendChild(wifiName);
        wifiInterface.appendChild(isSecure);

        return wifiInterface;
    }

    connect() {

    }
}