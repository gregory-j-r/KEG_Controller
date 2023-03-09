var listening_for_wifi_confirm = 0;
var WIFI_CH;
var hosted_page = "";
const github_bin_address = "https://github.com/gregory-j-r/KEG_Controller/tree/main/KEG_Arduino";
var wifiInterval;


function enableWifiUpload(){
    if(password_correct){
        showPopup();
    }
    else{
        console.log("Enter Password First");
        document.getElementById("on screen information").innerHTML = "Enter Password First";
    }
}

function setupListenWifiEstablish(){
    BLE_Server.getPrimaryService("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
    .then(service => {
        return service.getCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8");
    })
    .then(characteristic => {
        if (characteristic.properties.notify){
            WIFI_CH = characteristic;
            characteristic.addEventListener("characteristicvaluechanged",handleNewWifiData);
            characteristic.startNotifications();
            console.log("Wifi Setup Notifications enabled");
        }
        return 0;
    })
    .catch(error => { console.error(error); });
}

function handleNewWifiData(event){
    const value = event.target.value;
    var enc = new TextDecoder("utf-8");
    var reading = enc.decode(value);
    var split_reading = reading.split(".");
    
    if(split_reading.length == 4){
        console.log("Wifi Server Hosted At " + reading);
        hosted_page = reading;
        document.getElementById("on screen information").innerHTML = "Wifi Server Hosted At " + hosted_page;
        WIFI_CH.stopNotifications();
        WIFI_CH.removeEventListener("characteristicvaluechanged",handleNewPassMSG);
    }
    else{
        console.log(reading);
        if(reading.split(",").length<=1){
            if(reading == "NO"){
                document.getElementById("on screen information").innerHTML = "Wifi credentials incorrect";
            }
        }
    }
}

function redirectPage(){
    if(hosted_page != ""){
        window.open("http://"+hosted_page, '_blank').focus();
    }
}



function doneWifiCredentialCollection() { 
    document.getElementById("popup").style.display = "none";
    document.getElementById("popup").removeEventListener("keyup",handleEnterWifi);
    const SSID = document.getElementById("ssid").value;
    const WiFiPass = document.getElementById("pass").value;
    document.getElementById("ssid").value = "";
    document.getElementById("pass").value = "";
    //DO STUFF WITH PASSWORD HERE

    if(SSID!=null && WiFiPass!=null && SSID!="" && WiFiPass!=""){
        const msg = "W/" + SSID + "/" + WiFiPass;
        // console.log(msg);
        setupListenWifiEstablish();
        sendMSG(msg);
        document.getElementById("on screen information").innerHTML = "Building OTA Update Site ";
        wifiNotificationInter();
        setTimeout(() => {
            sendMSG("Standby");
            // window.open(github_bin_address, '_blank').focus();
          }, 1000);
    }
};

function showPopup() {
     document.getElementById("popup").style.display = "block";
     document.getElementById("popup").addEventListener("keyup", handleEnterWifi);
}

function handleEnterWifi(event){
    event.preventDefault();
    if (event.key === "Enter") {
        doneWifiCredentialCollection();
    }
}


function toggleHideWifiPass(){
    const pass_input = document.getElementById("pass");
    if (pass_input.getAttribute("type") === "password") {
      pass_input.setAttribute("type", "text");
    } else {
      pass_input.setAttribute("type", "password");
    }
}


function updateWifiLoadingMsg(){
    const str = document.getElementById("on screen information").innerHTML;
    if(str.substring(0,12) == "Building OTA"){
        switch(str.length){
            case(25):
                document.getElementById("on screen information").innerHTML = str+". ";
                break;
            case(27):
                document.getElementById("on screen information").innerHTML = str+". ";
                break;
            case(29):
                document.getElementById("on screen information").innerHTML = str+".";
                break;
            case(30):
                document.getElementById("on screen information").innerHTML = str.substring(0,25);
                break;
        }
    }
    else{
        clearInterval(wifiInterval);
    }
}

function wifiNotificationInter(){
    wifiInterval = setInterval(updateWifiLoadingMsg, 1000);
}


