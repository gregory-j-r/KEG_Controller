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
                    characteristic.addEventListener("characteristicvaluechanged",handleNewAnalogData);
                    characteristic.startNotifications();
                    console.log("Analog Notifications enabled");
                    setTimeout(() => {
                        sendMSG("A");
                      }, 1000);
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
    // else if(str3.length == 179){
    //     var AnalogCalibValues = enc.decode(value).split(':');
    //     for(let i=0;i<8;i++){
    //         Curr_AX_Cal_Vals[i] = AnalogCalibValues[0].split(',')[i];
    //     }
    //     for(let i=0;i<8;i++){
    //         Curr_AY_Cal_Vals[i] = AnalogCalibValues[1].split(',')[i];
    //     }
    //     for(let i=0;i<8;i++){
    //         Curr_CX_Cal_Vals[i] = AnalogCalibValues[2].split(',')[i];
    //     }
    //     for(let i=0;i<8;i++){
    //         Curr_CY_Cal_Vals[i] = AnalogCalibValues[3].split(',')[i];
    //     }
    //     console.log("Got Analog Calibration Values");
    //     document.getElementById("on screen information").innerHTML = "Got Analog Calibration Values";
    // }
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
        document.getElementById("on screen information").innerHTML = "Got Analog Deadzone Values";
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
    setCurrentCalVals();

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

    msg = msg + calArrayToMSG(AX_Cal_Vals,AY_Cal_Vals) + ":";
    msg = msg + calArrayToMSG(CX_Cal_Vals,CY_Cal_Vals);

    sendMSG(msg);
}

function saveCalibValues(){
    sendMSG("SAC");
}


function requestAnalogCalibration(){
    if(get_current_cal_flag == 1){
        sendMSG("RAC");
        get_current_cal_flag = 2;
    }
    else{
        if(get_current_cal_flag == 2){
            sendMSG("A");
            get_current_cal_flag = 1;
        }
    }
}



function editDeadzones(){
    // read input from user
    const ASXDStr = window.prompt("Enter Analog Stick X-Axis Deadzone\nformat: low, high, value\nwhere [low,high] --> value\nrecommended low<118, high>136, value = 127").split(",");
    const ASYDStr = window.prompt("Enter Analog Stick Y-Axis Deadzone\nformat: low, high, value\nwhere [low,high] --> value\nrecommended low<118, high>136, value = 127").split(",");
    const CSXDStr = window.prompt("Enter C-Stick X-Axis Deadzone\nformat: low, high, value\nwhere [low,high] --> value\nrecommended low<118, high>136, value = 127").split(",");
    const CSYDStr = window.prompt("Enter C-Stick Y-Axis Deadzone\nformat: low, high, value\nwhere [low,high] --> value\nrecommended low<118, high>136, value = 127").split(",");
    console.log("Deadzones set to:");
    // convert to integer
    var l = parseInt(ASXDStr[0]);
    var h = parseInt(ASXDStr[1]);
    var v = parseInt(ASXDStr[2]);
    // make sure all inputs read ok
    if(!isNaN(l)&&!isNaN(h)&&!isNaN(v)){
        if(l>h){
            console.log("Analog X low bound of " + l + " is larger than the high bound of " + h);
        }
        else if(v>h || v<l){
            console.log("Analog X deadzone value of " + v + " is not within the given bounds [" + l + "," + h +"]");
        }
        else if(l <0 || h > 255){
            console.log("Analog X deadzone bounds [" + l + "," + h +"] exceed controller limits [0,255]");
        }
        else{
            AnalogStickXDeadzone.low = l;
            AnalogStickXDeadzone.high = h;
            AnalogStickXDeadzone.value = v;
            console.log("AX = " + l + ", " + h + ", " + v);
        }
    }
    else{
        console.log("One or more Analog Stick X deadzone values were not integers");
    }
    // repeat
    l = parseInt(ASYDStr[0]);
    h = parseInt(ASYDStr[1]);
    v = parseInt(ASYDStr[2]);
    if(!isNaN(l)&&!isNaN(h)&&!isNaN(v)){
        if(l>h){
            console.log("Analog Y low bound of " + l + " is larger than the high bound of " + h);
        }
        else if(v>h || v<l){
            console.log("Analog Y deadzone value of " + v + " is not within the given bounds [" + l + "," + h +"]");
        }
        else if(l <0 || h > 255){
            console.log("Analog Y deadzone bounds [" + l + "," + h +"] exceed controller limits [0,255]");
        }
        else{
            AnalogStickYDeadzone.low = l;
            AnalogStickYDeadzone.high = h;
            AnalogStickYDeadzone.value = v;
            console.log("AY = " + l + ", " + h + ", " + v);
        }
    }
    else{
        console.log("One or more Analog Stick Y deadzone values were not integers");
    }
    l = parseInt(CSXDStr[0]);
    h = parseInt(CSXDStr[1]);
    v = parseInt(CSXDStr[2]);
    if(!isNaN(l)&&!isNaN(h)&&!isNaN(v)){
        if(l>h){
            console.log("C-Stick X low bound of " + l + " is larger than the high bound of " + h);
        }
        else if(v>h || v<l){
            console.log("C-Stick X deadzone value of " + v + " is not within the given bounds [" + l + "," + h +"]");
        }
        else if(l <0 || h > 255){
            console.log("C-Stick X deadzone bounds [" + l + "," + h +"] exceed controller limits [0,255]");
        }
        else{
            CStickXDeadzone.low = l;
            CStickXDeadzone.high = h;
            CStickXDeadzone.value = v;
            console.log("CX = " + l + ", " + h + ", " + v);
        }
    }
    else{
        console.log("One or more C-Stick X deadzone values were not integers");
    }
    l = parseInt(CSYDStr[0]);
    h = parseInt(CSYDStr[1]);
    v = parseInt(CSYDStr[2]);
    if(!isNaN(l)&&!isNaN(h)&&!isNaN(v)){
        if(l>h){
            console.log("C-Stick Y low bound of " + l + " is larger than the high bound of " + h);
        }
        else if(v>h || v<l){
            console.log("C-Stick Y deadzone value of " + v + " is not within the given bounds [" + l + "," + h +"]");
        }
        else if(l <0 || h > 255){
            console.log("C-Stick Y deadzone bounds [" + l + "," + h +"] exceed controller limits [0,255]");
        }
        else{
            CStickYDeadzone.low = l;
            CStickYDeadzone.high = h;
            CStickYDeadzone.value = v;
            console.log("CY = " + l + ", " + h + ", " + v);
        }
    }
    else{
        console.log("One or more C-Stick Y deadzone values were not integers");
    }
    sendStickDeadzones();
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