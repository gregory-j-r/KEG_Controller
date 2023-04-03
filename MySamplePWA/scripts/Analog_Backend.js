var currentAX = -1;
var currentAY = -1;
var currentCX = -1;
var currentCY = -1;
var currentLT = -1;
var currentRT = -1;

var storageCounter = 0;

var AX_Cal_Vals = [0,0,0,0,0,0,0,0,0];
var AY_Cal_Vals = [0,0,0,0,0,0,0,0,0];
var CX_Cal_Vals = [0,0,0,0,0,0,0,0,0];
var CY_Cal_Vals = [0,0,0,0,0,0,0,0,0];

var Curr_AX_Cal_Vals = [0,0,0,0,0,0,0];
var Curr_AY_Cal_Vals = [0,0,0,0,0,0,0];
var Curr_CX_Cal_Vals = [0,0,0,0,0,0,0];
var Curr_CY_Cal_Vals = [0,0,0,0,0,0,0];

var ready_toggle = 0;

var ANALOG_CH;

let numCalPoints = 9;

const switchToCStick = 9;
const doneCalibratingVal = 18;

var storeValueButtonFlag = 0;

// Defines the deadzone class
// a deadzone maps all values between the lower and upper bounds (low and high) inclusive to the specified value
// value should be between low and high inclusive
class Deadzone {
    constructor(low, high, value) {
      this.low = low;
      this.high = high;
      this.value = value;
    }
  }

var AnalogStickXDeadzone = new Deadzone(117,137,127);
var AnalogStickYDeadzone = new Deadzone(117,137,127);
var CStickXDeadzone = new Deadzone(117,137,127);
var CStickYDeadzone = new Deadzone(117,137,127);


function requestAnalogReadings(){
    // if(password_correct){
        if(in_window_index != 1){
            finishedDigitalSettings();
            finishedDebug();
            inter();
            in_window_index = 1;
            sendMSG("RAC");
            // var msg = "A";
            // sendMSG(msg);
            console.log("Requesting analog data");
            BLE_Server.getPrimaryService("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
            .then(service => {
                return service.getCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8");
            })
            .then(characteristic => {
                if (characteristic.properties.notify){
                    ANALOG_CH = characteristic;
                    setTimeout(() => {
                        characteristic.addEventListener("characteristicvaluechanged",handleNewAnalogData);
                        characteristic.startNotifications();

                        console.log("Analog Notifications enabled");
                        setTimeout(() => {
                            sendMSG("RDC");
                            console.log("Analog Notifications enabled");
                            setTimeout(() => {
                                sendMSG("RTT");
                                setTimeout(() => {
                                    sendMSG("A");
                                }, 1000);
                            }, 1000);
                        }, 1000); 
                    },1000);
                }
                return 0;
            })
            .catch(error => { console.error(error); });
        }
    // }
    // else{
    //     console.log("Enter Password First");
    //     document.getElementById("on screen information").innerHTML = "Enter Password First";
    // }
}

async function handleNewAnalogData(event){
    const value = event.target.value;
    var enc = new TextDecoder("utf-8");
    var str3 = enc.decode(value);
    if(str3.length == 29){
        var AnalogValues = enc.decode(value).split(',');
        currentAX = parseInt(AnalogValues[0]);
        currentAY = parseInt(AnalogValues[1]);
        currentCX = parseInt(AnalogValues[2]);
        currentCY = parseInt(AnalogValues[3]);
        currentLT = parseInt(AnalogValues[4]);
        currentRT = parseInt(AnalogValues[5]);
    }
    else if(str3.length == 139){
        var AnalogCalibValues = enc.decode(value).split(':');
        for(let i=0;i<7;i++){
            Curr_AX_Cal_Vals[i] = AnalogCalibValues[0].split(',')[i];
        }
        for(let i=0;i<7;i++){
            Curr_AY_Cal_Vals[i] = AnalogCalibValues[1].split(',')[i];
        }
        for(let i=0;i<7;i++){
            Curr_CX_Cal_Vals[i] = AnalogCalibValues[2].split(',')[i];
        }
        for(let i=0;i<7;i++){
            Curr_CY_Cal_Vals[i] = AnalogCalibValues[3].split(',')[i];
        }
        console.log("Got Analog Calibration Values");
        console.log(AnalogCalibValues);
        document.getElementById("on screen information").innerHTML = "Got Analog Calibration Values";
    }
    else if(str3.length == 47){
        var AnalogDeadzoneValues = enc.decode(value).split(':');
        AnalogStickXDeadzone.low = AnalogDeadzoneValues[0].split(',')[0];
        AnalogStickXDeadzone.high = AnalogDeadzoneValues[0].split(',')[1];
        AnalogStickXDeadzone.value = AnalogDeadzoneValues[0].split(',')[2];

        AnalogStickYDeadzone.low = AnalogDeadzoneValues[1].split(',')[0];
        AnalogStickYDeadzone.high = AnalogDeadzoneValues[1].split(',')[1];
        AnalogStickYDeadzone.value = AnalogDeadzoneValues[1].split(',')[2];

        CStickXDeadzone.low = AnalogDeadzoneValues[2].split(',')[0];
        CStickXDeadzone.high = AnalogDeadzoneValues[2].split(',')[1];
        CStickXDeadzone.value = AnalogDeadzoneValues[2].split(',')[2];

        CStickYDeadzone.low = AnalogDeadzoneValues[3].split(',')[0];
        CStickYDeadzone.high = AnalogDeadzoneValues[3].split(',')[1];
        CStickYDeadzone.value = AnalogDeadzoneValues[3].split(',')[2];
        
        console.log("Got Analog Deadzone Values");
        console.log("AX Deadzone = " + AnalogStickXDeadzone.low + ", " + AnalogStickXDeadzone.high + ", " + AnalogStickXDeadzone.value);
        console.log("AY Deadzone = " + AnalogStickYDeadzone.low + ", " + AnalogStickYDeadzone.high + ", " + AnalogStickYDeadzone.value);
        console.log("CX Deadzone = " + CStickXDeadzone.low + ", " + CStickXDeadzone.high + ", " + CStickXDeadzone.value);
        console.log("CY Deadzone = " + CStickYDeadzone.low + ", " + CStickYDeadzone.high + ", " + CStickYDeadzone.value);
        document.getElementById("on screen information").innerHTML = "Got Analog Deadzone Values";
    }
    else if(str3.length == 2){
        var TriggerToggling = enc.decode(value);
        L_Trigger_on = parseInt(TriggerToggling[0]);
        R_Trigger_on = parseInt(TriggerToggling[1]);
        console.log("Got trigger toggling = " + L_Trigger_on + "," + R_Trigger_on);
    }
    else{
        console.log("Got analog message: ")
        console.log(str3);
    }
}

function storeValue(){
    // console.log("store value button clicked");
    
    // if(storageCounter < numCalPoints){
    //     AX_Cal_Vals[storageCounter] = currentAX;
    // } else if(storageCounter < numCalPoints*2){
    //     AY_Cal_Vals[storageCounter-numCalPoints] = currentAY;
    // } else if(storageCounter < numCalPoints*3){
    //     CX_Cal_Vals[storageCounter-numCalPoints*2] = currentCX;
    // } else{
    //     CY_Cal_Vals[storageCounter-numCalPoints*3] = currentCY;
    // }

    if(storageCounter < switchToCStick){
        AX_Cal_Vals[storageCounter] = currentAX;
        AY_Cal_Vals[storageCounter] = currentAY;
        console.log("AX = " + currentAX);
        console.log("AY = " + currentAY);
        console.log("");
    }
    else{
        CX_Cal_Vals[storageCounter-switchToCStick] = currentCX;
        CY_Cal_Vals[storageCounter-switchToCStick] = currentCY;
        console.log("CX = " + currentCX);
        console.log("CY = " + currentCY);
        console.log("");
    }

    storageCounter = storageCounter +1;
    

    // console.log("Storage counter = " + storageCounter);
    // for(let i=0;i<numCalPoints;i++){
    //     console.log(AX_Cal_Vals[i])
    // }
    // console.log("");
    // for(let i=0;i<numCalPoints;i++){
    //     console.log(AY_Cal_Vals[i])
    // }
    // console.log("");
    // for(let i=0;i<numCalPoints;i++){
    //     console.log(CX_Cal_Vals[i])
    // }
    // console.log("");
    // for(let i=0;i<numCalPoints;i++){
    //     console.log(CY_Cal_Vals[i])
    // }
    // console.log("");

    if(storageCounter >= doneCalibratingVal+1){
        doneCalibration();
    }

}

function redoStorageButtonClicked(){
    if(storageCounter > 0){
        storageCounter = storageCounter-1;
    }
    console.log("Storage counter = " + storageCounter);
    if(storageCounter==0){
        redo_last_store_flag = 0;
    }
}

function skipToCStick(){
    storageCounter = switchToCStick;
}

function doneCalibration(){
    storageCounter = 0;
    store_val_flag = 0;
    done_calib_flag = 0;
    deadzones_flag = 0;
    redo_last_store_flag = 0;
    toggle_triggers_flag = 1;
    toggle_stick_raw_flag = 1;
    
    // don't ;et trigger toggle buttons get hit for 1 sec
    ready_toggle = 1;
    setTimeout(() => {
        ready_toggle = 0;
      }, 1);
}

function finishedCalibration(){
    analog_stick_flag = 0;
    c_stick_flag = 0;
    get_current_cal_flag = 0;
    send_calib_flag = 0;
    save_calib_flag = 0;
    finished_calib_flag = 0;
    store_val_flag = 0;
    done_calib_flag = 0;
    redo_last_store_flag = 0;
    deadzones_flag = 0;
    trigger_flag = 0;
    toggle_triggers_flag = 0;
    toggle_stick_raw_flag = 0;

    in_window_index = 0;

    try{
        ANALOG_CH.stopNotifications();
        ANALOG_CH.removeEventListener("characteristicvaluechanged",handleNewAnalogData);
    }
    catch(err){
        console.log(err.message);
    }
}



function checkAndRemoveChild(child){
    if(document.contains(child)){
        child.parentNode.removeChild(child);
    }
}

function sendStickCalibration(){
    var msg = "";

    // msg = msg + String(AX_Cal_Vals[0]).padStart(4, '0') + "," + String(AX_Cal_Vals[1]).padStart(4, '0') + "," + String(AX_Cal_Vals[2]).padStart(4, '0') + "," + String(AX_Cal_Vals[3]).padStart(4, '0') + "," + String(AX_Cal_Vals[4]).padStart(4, '0') + "," + String(AX_Cal_Vals[5]).padStart(4, '0') + "," + String(AX_Cal_Vals[6]).padStart(4, '0') + "," + String(AX_Cal_Vals[7]).padStart(4, '0') + "," + String(AX_Cal_Vals[8]).padStart(4, '0') + ":";
    // msg = msg + String(AY_Cal_Vals[0]).padStart(4, '0') + "," + String(AY_Cal_Vals[1]).padStart(4, '0') + "," + String(AY_Cal_Vals[2]).padStart(4, '0') + "," + String(AY_Cal_Vals[3]).padStart(4, '0') + "," + String(AY_Cal_Vals[4]).padStart(4, '0') + "," + String(AY_Cal_Vals[5]).padStart(4, '0') + "," + String(AY_Cal_Vals[6]).padStart(4, '0') + "," + String(AY_Cal_Vals[7]).padStart(4, '0') + "," + String(AY_Cal_Vals[8]).padStart(4, '0')+ ":";
    // msg = msg + String(CX_Cal_Vals[0]).padStart(4, '0') + "," + String(CX_Cal_Vals[1]).padStart(4, '0') + "," + String(CX_Cal_Vals[2]).padStart(4, '0') + "," + String(CX_Cal_Vals[3]).padStart(4, '0') + "," + String(CX_Cal_Vals[4]).padStart(4, '0') + "," + String(CX_Cal_Vals[5]).padStart(4, '0') + "," + String(CX_Cal_Vals[6]).padStart(4, '0') + "," + String(CX_Cal_Vals[7]).padStart(4, '0') + "," + String(CX_Cal_Vals[8]).padStart(4, '0')+ ":";
    // msg = msg + String(CY_Cal_Vals[0]).padStart(4, '0') + "," + String(CY_Cal_Vals[1]).padStart(4, '0') + "," + String(CY_Cal_Vals[2]).padStart(4, '0') + "," + String(CY_Cal_Vals[3]).padStart(4, '0') + "," + String(CY_Cal_Vals[4]).padStart(4, '0') + "," + String(CY_Cal_Vals[5]).padStart(4, '0') + "," + String(CY_Cal_Vals[6]).padStart(4, '0') + "," + String(CY_Cal_Vals[7]).padStart(4, '0') + "," + String(CY_Cal_Vals[8]).padStart(4, '0');
    // sendMSG(msg);

    var sum = 0;
    for(let i=0; i<7; i++){
        sum+=AX_Cal_Vals[i];
        sum+=AY_Cal_Vals[i];
        sum+=CX_Cal_Vals[i];
        sum+=CY_Cal_Vals[i];
    }
    if(sum>0){
        msg = msg + calArrayToMSG(AX_Cal_Vals,AY_Cal_Vals) + ":";
        msg = msg + calArrayToMSG(CX_Cal_Vals,CY_Cal_Vals);
        sendMSG(msg);
    }
    else{
        console.log("No stick calibration changes made");
    }

}

function saveCalibValues(){
    sendMSG("SAC");
}


function requestAnalogCalibration(){
    if(get_current_cal_flag == 1){
        sendMSG("RAC");
        setTimeout(() => {
            sendMSG("RDC");
            console.log("Analog Notifications enabled");
            setTimeout(() => {
                sendMSG("RTT");
                setTimeout(() => {
                    sendMSG("A");
                }, 1000);
            }, 1000);
        }, 2000); 
        // get_current_cal_flag = 2;
    }
    else{
        if(get_current_cal_flag == 2){
            sendMSG("A");
            get_current_cal_flag = 1;
        }
    }
}


function showDeadzonePopup(){
    if(storageCounter < 9 && store_val_flag){
        document.getElementById("AXDeadLow").value = AnalogStickXDeadzone.low.toString()
        document.getElementById("AXDeadHigh").value = AnalogStickXDeadzone.high.toString()
        document.getElementById("AXDeadVal").value = AnalogStickXDeadzone.value.toString();
        document.getElementById("AYDeadLow").value = AnalogStickYDeadzone.low.toString()
        document.getElementById("AYDeadHigh").value = AnalogStickYDeadzone.high.toString()
        document.getElementById("AYDeadVal").value = AnalogStickYDeadzone.value.toString();

        document.getElementById("analog_deadzone_popup").style.display = "block";
        document.getElementById("analog_deadzone_popup").addEventListener("keyup", handleEnterChangeADeadzone);
    }
    else{
        document.getElementById("CXDeadLow").value = CStickXDeadzone.low.toString()
        document.getElementById("CXDeadHigh").value = CStickXDeadzone.high.toString()
        document.getElementById("CXDeadVal").value = CStickXDeadzone.value.toString();
        document.getElementById("CYDeadLow").value = CStickYDeadzone.low.toString()
        document.getElementById("CYDeadHigh").value = CStickYDeadzone.high.toString()
        document.getElementById("CYDeadVal").value = CStickYDeadzone.value.toString();

        document.getElementById("CStick_deadzone_popup").style.display = "block";
        document.getElementById("CStick_deadzone_popup").addEventListener("keyup", handleEnterChangeCDeadzone);
    }
}

function doneAnalogDeadzone(){
    const AXDL = parseInt(document.getElementById("AXDeadLow").value);
    const AXDH = parseInt(document.getElementById("AXDeadHigh").value);
    const AXDV = parseInt(document.getElementById("AXDeadVal").value);
    const AYDL = parseInt(document.getElementById("AYDeadLow").value);
    const AYDH = parseInt(document.getElementById("AYDeadHigh").value);
    const AYDV = parseInt(document.getElementById("AYDeadVal").value);

    var checksum = 0;

    if(checkDeadzones(AXDL,AXDH,AXDV)){
        AnalogStickXDeadzone.low = AXDL;
        AnalogStickXDeadzone.high = AXDH;
        AnalogStickXDeadzone.value = AXDV;
        checksum++;
    }
    if(checkDeadzones(AYDL,AYDH,AYDV)){
        AnalogStickYDeadzone.low = AYDL;
        AnalogStickYDeadzone.high = AYDH;
        AnalogStickYDeadzone.value = AYDV;
        checksum++;
    }
    if(checksum == 2){
        closeAnalogDeadzone();
    }
    else{
        document.getElementById("AnalogDeadzoneMSG").innerHTML = "One or more deadzone ranges is invalid";
    }
}

function closeAnalogDeadzone(){
    document.getElementById("analog_deadzone_popup").style.display = "none";
    document.getElementById("analog_deadzone_popup").removeEventListener("keyup",handleEnterChangeADeadzone);
}

function doneCStickDeadzone(){
    const CXDL = parseInt(document.getElementById("CXDeadLow").value);
    const CXDH = parseInt(document.getElementById("CXDeadHigh").value);
    const CXDV = parseInt(document.getElementById("CXDeadVal").value);
    const CYDL = parseInt(document.getElementById("CYDeadLow").value);
    const CYDH = parseInt(document.getElementById("CYDeadHigh").value);
    const CYDV = parseInt(document.getElementById("CYDeadVal").value);

    var checksum = 0;

    if(checkDeadzones(CXDL,CXDH,CXDV)){
        CStickXDeadzone.low = CXDL;
        CStickXDeadzone.high = CXDH;
        CStickXDeadzone.value = CXDV;
        checksum++;
    }
    if(checkDeadzones(CYDL,CYDH,CYDV)){
        CStickYDeadzone.low = CYDL;
        CStickYDeadzone.high = CYDH;
        CStickYDeadzone.value = CYDV;
        checksum++;
    }
    if(checksum == 2){
        closeCStickDeadzone();
    }
    else{
        document.getElementById("CStickDeadzoneMSG").innerHTML = "One or more deadzone ranges is invalid";
    }
}

function closeCStickDeadzone(){
    document.getElementById("CStick_deadzone_popup").style.display = "none";
    document.getElementById("CStick_deadzone_popup").removeEventListener("keyup",handleEnterChangeCDeadzone);
}

function handleEnterChangeADeadzone(event){
    event.preventDefault();
    if (event.key === "Enter") {
        doneAnalogDeadzone();
    }
}

function handleEnterChangeCDeadzone(event){
    event.preventDefault();
    if (event.key === "Enter") {
        doneCStickDeadzone();
    }
}

function checkDeadzones(l,h,v){
    if(!isNaN(l)&&!isNaN(h)&&!isNaN(v)){
        if(l>h){
            console.log("low bound of " + l + " is larger than the high bound of " + h);
        }
        else if(v>h || v<l){
            console.log("deadzone value of " + v + " is not within the given bounds [" + l + "," + h +"]");
        }
        else if(l <0 || h > 255){
            console.log("deadzone bounds [" + l + "," + h +"] exceed controller limits [0,255]");
        }
        else{
            console.log(l + ", " + h + ", " + v);
            return 1;
        }
    }
    else{
        console.log("One or more Analog Stick X deadzone values were not integers");
    }
    return 0;
}

function sendStickDeadzones(){
    console.log("Sending Stick Deadzone Settings");

    var msg = "";
    msg = msg + String(AnalogStickXDeadzone.low).padStart(3, '0') + "," + String(AnalogStickXDeadzone.high).padStart(3, '0') + "," + String(AnalogStickXDeadzone.value).padStart(3, '0') + ":";
    msg = msg + String(AnalogStickYDeadzone.low).padStart(3, '0') + "," + String(AnalogStickYDeadzone.high).padStart(3, '0') + "," + String(AnalogStickYDeadzone.value).padStart(3, '0') + ":";
    msg = msg + String(CStickXDeadzone.low).padStart(3, '0') + "," + String(CStickXDeadzone.high).padStart(3, '0') + "," + String(CStickXDeadzone.value).padStart(3, '0') + ":";
    msg = msg + String(CStickYDeadzone.low).padStart(3, '0') + "," + String(CStickYDeadzone.high).padStart(3, '0') + "," + String(CStickYDeadzone.value).padStart(3, '0');
    
    sendMSG(msg);
    // next line sets current cal, should remove later as current cal should be received from controller
}

function saveStickDeadzones(){
    sendMSG("SSD");
}

// makse the callibration message to send to controller
// the diagonal notches need to be sorted for their respective sides
function calArrayToMSG(XCalArr,YCalArr){
    // West Notch1 Notch2 Neutch Notch3 Notch4 East
    var neutch = XCalArr[0];
    var west = XCalArr[1];
    var northWest = XCalArr[2];
    var northEast = XCalArr[4];
    var east = XCalArr[5];
    var southEast = XCalArr[6];
    var southWest = XCalArr[8];

    var Xpoints = [west,northWest,southWest,neutch,northEast,southEast,east];

    var msg = "";

    if(west < east){
        Xpoints.sort(function(a, b){return a - b});
    }
    else{
        Xpoints.sort(function(a, b){return b - a});
    }

    for(let i = 0; i<7; i++){
        msg = msg + String(Xpoints[i]).padStart(4, '0');
        if(i != 6){
            msg = msg + ",";
        }
    }
    msg = msg + ":";

    // South Notch5 Notch6 Neutch Notch7 Notch8 North
    neutch = YCalArr[0];
    northWest = YCalArr[2];
    var north = YCalArr[3];
    northEast = YCalArr[4];
    southEast = YCalArr[6];
    var south = YCalArr[7]
    southWest = YCalArr[8];

    var Ypoints = [south,southWest,southEast,neutch,northWest,northEast,north];

    if(south < north){
        Ypoints.sort(function(a, b){return a - b});
    }
    else{
        Ypoints.sort(function(a, b){return b - a});
    }

    for(let i = 0; i<7; i++){
        msg = msg + String(Ypoints[i]).padStart(4, '0');
        if(i != 6){
            msg = msg + ",";
        }
    }
    
    return msg;

}


function setCurrentCalVals(){
    var neutch = AX_Cal_Vals[0];
    var west = AX_Cal_Vals[1];
    var northWest = AX_Cal_Vals[2];
    var northEast = AX_Cal_Vals[4];
    var east = AX_Cal_Vals[5];
    var southEast = AX_Cal_Vals[6];
    var southWest = AX_Cal_Vals[8];

    Curr_AX_Cal_Vals [0] = west;
    Curr_AX_Cal_Vals [1] = northWest;
    Curr_AX_Cal_Vals [2] = southWest;
    Curr_AX_Cal_Vals [3] = neutch;
    Curr_AX_Cal_Vals [4] = northEast;
    Curr_AX_Cal_Vals [5] = southEast;
    Curr_AX_Cal_Vals [6] = east;

    if(west < east){
        Curr_AX_Cal_Vals.sort(function(a, b){return a - b});
    }
    else{
        Curr_AX_Cal_Vals.sort(function(a, b){return b - a});
    }

    

    neutch = AY_Cal_Vals[0];
    northWest = AY_Cal_Vals[2];
    var north = AY_Cal_Vals[3];
    northEast = AY_Cal_Vals[4];
    southEast = AY_Cal_Vals[6];
    var south = AY_Cal_Vals[7];
    southWest = AY_Cal_Vals[8];

    Curr_AY_Cal_Vals[0] = south;
    Curr_AY_Cal_Vals[1] = southWest;
    Curr_AY_Cal_Vals[2] = southEast;
    Curr_AY_Cal_Vals[3] = neutch;
    Curr_AY_Cal_Vals[4] = northWest;
    Curr_AY_Cal_Vals[5] = northEast;
    Curr_AY_Cal_Vals[6] = north;

    if(south < north){
        Curr_AY_Cal_Vals.sort(function(a, b){return a - b});
    }
    else{
        Curr_AY_Cal_Vals.sort(function(a, b){return b - a});
    }

    // now the c stick
    neutch = CX_Cal_Vals[0];
    west = CX_Cal_Vals[1];
    northWest = CX_Cal_Vals[2];
    northEast = CX_Cal_Vals[4];
    east = CX_Cal_Vals[5];
    southEast = CX_Cal_Vals[6];
    southWest = CX_Cal_Vals[8];

    Curr_CX_Cal_Vals [0] = west;
    Curr_CX_Cal_Vals [1] = northWest;
    Curr_CX_Cal_Vals [2] = southWest;
    Curr_CX_Cal_Vals [3] = neutch;
    Curr_CX_Cal_Vals [4] = northEast;
    Curr_CX_Cal_Vals [5] = southEast;
    Curr_CX_Cal_Vals [6] = east;

    if(west < east){
        Curr_CX_Cal_Vals.sort(function(a, b){return a - b});
    }
    else{
        Curr_CX_Cal_Vals.sort(function(a, b){return b - a});
    }

    neutch = CY_Cal_Vals[0];
    northWest = CY_Cal_Vals[2];
    north = CY_Cal_Vals[3];
    northEast = CY_Cal_Vals[4];
    southEast = CY_Cal_Vals[6];
    south = CY_Cal_Vals[7];
    southWest = CY_Cal_Vals[8];

    Curr_CY_Cal_Vals[0] = south;
    Curr_CY_Cal_Vals[1] = southWest;
    Curr_CY_Cal_Vals[2] = southEast;
    Curr_CY_Cal_Vals[3] = neutch;
    Curr_CY_Cal_Vals[4] = northWest;
    Curr_CY_Cal_Vals[5] = northEast;
    Curr_CY_Cal_Vals[6] = north;

    if(south < north){
        Curr_CY_Cal_Vals.sort(function(a, b){return a - b});
    }
    else{
        Curr_CY_Cal_Vals.sort(function(a, b){return b - a});
    }
    console.log("Current ax cal = " + Curr_AX_Cal_Vals);
    console.log("Current ay cal = " + Curr_AY_Cal_Vals);
    console.log("Current cx cal = " + Curr_CX_Cal_Vals);
    console.log("Current cx cal = " + Curr_CY_Cal_Vals);

}