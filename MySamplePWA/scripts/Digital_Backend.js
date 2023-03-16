
var DIGITAL_CH;

const Digital_Button_Map = new Map();
const Toggle_Map = new Map();
setDefaultMapping();

var current_buttons_in = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];


var first_clicked = "";
var second_clicked = "";
var num_clicked = 0;
var what_button_msg_flag = 0;

function requestDigitalReadings(){
    // if(password_correct){
        if(in_window_index != 2){
            finishedCalibration();
            DigitalInter();
            in_window_index = 2;
            var msg = "RBM";
            sendMSG(msg);
            console.log("Requesting digital data");
            BLE_Server.getPrimaryService("4fafc201-1fb5-459e-8fcc-c5c9c331914b")
            .then(service => {
                return service.getCharacteristic("beb5483e-36e1-4688-b7f5-ea07361b26a8");
            })
            .then(characteristic => {
                if (characteristic.properties.notify){
                    DIGITAL_CH = characteristic;
                    characteristic.addEventListener("characteristicvaluechanged",handleNewDigitalData);
                    characteristic.startNotifications();
                    console.log("Digital Notifications enabled");
                    setTimeout(() => {
                        sendMSG("D");
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


async function handleNewDigitalData(event){
    const value = event.target.value;
    var enc = new TextDecoder("utf-8");
    var str4 = enc.decode(value);
    if(str4.length == 35){
        const split_read = str4.split(".");
        let keys =[ ...Digital_Button_Map.keys() ];
        for(let i=0; i<12; i++){
            Digital_Button_Map.set(keys[i],split_read[i][0]);
            Toggle_Map.set(Digital_Button_Map.get(keys[i]),split_read[i][1]);
        }
        // console.log([...Digital_Button_Map.entries()]);
        // console.log([...Toggle_Map.entries()]);
    }
    else if(str4.length == 16){
        for(let i = 0; i< 16; i++){
            current_buttons_in[i] = parseInt(str4[i]);
            setColourFlags();
        }
    }
    // console.log(str4);
}


function finishedDigitalSettings(){
    A_Button_flag = 0;
    B_Button_flag = 0;
    S_Button_flag = 0;
    D_Button_flag = 0;
    X_Button_flag = 0;
    Y_Button_flag = 0;
    Z_Button_flag = 0;
    L_Button_flag = 0;
    R_Button_flag = 0;

    Draw_Tags_Flag = 0;

    Read_Mapping_Button_Flag = 0;
    Send_Mapping_Button_Flag = 0;
    Save_Mapping_Button_Flag = 0;
    Done_Mapping_Button_Flag = 0;

    Redo_Last_Button_Flag = 0;
    Reset_Default_Mapping_Flag = 0;

    num_clicked = 0;
    first_clicked = "";
    second_clicked = "";

    in_window_index = 0;

    try{
        DIGITAL_CH.stopNotifications();
        DIGITAL_CH.removeEventListener("characteristicvaluechanged",handleNewDigitalData);
    }
    catch(err){
        console.log(err.message);
    }
}

function setDefaultMapping(){
    Digital_Button_Map.set("A","A");
    Digital_Button_Map.set("B","B");
    Digital_Button_Map.set("S","S");
    Digital_Button_Map.set("X","X");
    Digital_Button_Map.set("Y","Y");
    Digital_Button_Map.set("Z","Z");
    Digital_Button_Map.set("L","L");
    Digital_Button_Map.set("R","R");
    Digital_Button_Map.set("u","u");
    Digital_Button_Map.set("r","r");
    Digital_Button_Map.set("d","d");
    Digital_Button_Map.set("l","l");

    Toggle_Map.set("A","Y");
    Toggle_Map.set("B","Y");
    Toggle_Map.set("S","Y");
    Toggle_Map.set("X","Y");
    Toggle_Map.set("Y","Y");
    Toggle_Map.set("Z","Y");
    Toggle_Map.set("L","Y");
    Toggle_Map.set("R","Y");
    Toggle_Map.set("u","Y");
    Toggle_Map.set("r","Y");
    Toggle_Map.set("d","Y");
    Toggle_Map.set("l","Y");
}


function twoClicked(){
    if(first_clicked == second_clicked){
        toggleButton(first_clicked);
        what_button_msg_flag = 1;
    }
    else{
        swapButtons(first_clicked,second_clicked);
        what_button_msg_flag = 2;
    }
    console.log([...Digital_Button_Map.entries()]);
    console.log([...Toggle_Map.entries()]);
}

function redoLastButtonClick(){
    num_clicked = 0;
    first_clicked = "";
    Redo_Last_Button_Flag = 0;
}

function toggleButton(tag){
    const curr_but_val = Toggle_Map.get(tag);
    switch(curr_but_val){
        case "Y":
            Toggle_Map.set(tag,"N")
            break;
        case "N":
            Toggle_Map.set(tag,"Y");
            break
    }
}

function swapButtons(tag1,tag2){
    const map_val_1 = Digital_Button_Map.get(tag1);
    const map_val_2 = Digital_Button_Map.get(tag2);
    Digital_Button_Map.set(tag1,map_val_2);
    Digital_Button_Map.set(tag2,map_val_1);
}

function sendButtonMapping(){
    var msg = "";
    let keys =[ ...Digital_Button_Map.keys() ];
    for(let i=0; i<12; i++){
        msg = msg + Digital_Button_Map.get(keys[i]) + Toggle_Map.get(Digital_Button_Map.get(keys[i]));
        if(i!=11){
            msg = msg + ".";
        }
    }
    sendMSG(msg);
    setTimeout(() => {
        sendMSG("D");
    }, 1000);
}

function saveButtonMapping(){
    sendMSG("SBM"); // SBM = Save Button Mapping
    setTimeout(() => {
        sendMSG("D");
    }, 1000);
}

function requestButtonMapping(){
    sendMSG("RBM"); // RBM = Request Button Mapping
    setTimeout(() => {
        sendMSG("D");
    }, 1000);
}