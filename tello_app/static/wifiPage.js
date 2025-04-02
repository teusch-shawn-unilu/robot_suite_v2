export class WifiPage {
    constructor() {}

    start() {
        this.intervalId = window.setInterval(async function () {
            try {
                var response = await fetch("/get_wifi_networks");
                
                if (!response.ok) 
                    console.log("Issue fetching wifi networks");
                
                this.networks = await response.json();

                response = await fetch("/get_connected_network");

                if (!response.ok)
                    console.log("Issue fetching connected network");

                this.connected_network = await response.json();

                updateWifiInterface();

            } catch (error) {
                console.log("Error fetching wifi networks")
            }
        }, 10000);
    }

    stop() {
        clearInterval(this.intervalId);
        this.intervalId = null;
    }

    updateWifiInterface() {
        console.log(this.networks);
        console.log(this.connected_network);
    }

    connect() {

    }
}