// const analog_stick_cal_X_shifts = [0,-1/2,1/2,0,0,0,0,0,0,0,0,0,0];
// const analog_stick_cal_Y_shifts = [0,0,0,0,1/2,-1/2,0,0,0,0,0,0,0];

// const c_stick_cal_X_shifts = [0,0,0,0,0,0,0,-1/2,1/2,0,0,0,0];
// const c_stick_cal_Y_shifts = [0,0,0,0,0,0,0,0,0,0,1/2,-1/2,0];

const analog_stick_cal_X_shifts = [0, -1/2, -0.354,  0,    0.354, 1/2, 0.354,  0,   -0.354, 0,0,0,0,0,0,0,0,0,0];
const analog_stick_cal_Y_shifts = [0,  0,   -0.354, -1/2, -0.354, 0,   0.354,  1/2,  0.354, 0,0,0,0,0,0,0,0,0,0];

const c_stick_cal_X_shifts = [0,0,0,0,0,0,0,0,0,0, -1/2,   -0.354,  0,      0.354, 1/2,   0.354,  0,    -0.354, 0];
const c_stick_cal_Y_shifts = [0,0,0,0,0,0,0,0,0,0,  0,     -0.354, -1/2,   -0.354, 0,     0.354,  1/2,   0.354, 0];

var analog_stick_X;
var analog_stick_Y;

var c_stick_X;
var c_stick_Y;

var LT_pos = 0;
var RT_pos = 0;

// document.body.style.backgroundColor = "#495096";
document.body.style.backgroundImage = "linear-gradient(to right, " + "#343A70" + ", " + "#646CB7" + ")";

const Analog_Instructions = [
    "Click on the stick you would like to calibrate",
    "Place your controlers stick to the position shown",
    "then hit the Store Value button",
    "Repeat until finished, if you make a mistake",
    "simply hit Redo"
];

var drawInterval;

var analog_stick_bounding_rect = new Path2D();
var c_stick_bounding_rect = new Path2D();

const store_value_rect = new Path2D();
const redo_last_rect = new Path2D();
const done_calib_rect = new Path2D();
const get_current_cal_rect = new Path2D();
const send_calib_rect = new Path2D();
const save_calib_rect = new Path2D();
const finished_calib_rect = new Path2D();
const deadzones_rect = new Path2D();
const send_deadzones_rect = new Path2D();
const save_deadzones_rect = new Path2D();
const toggle_L_trigger_rect = new Path2D();
const toggle_R_trigger_rect = new Path2D();
const toggle_stick_raw_rect = new Path2D();

const left_trig_base_path = new Path2D();
const right_trig_base_path = new Path2D();

const button_w = 160;
const button_h = 65;

store_value_rect.rect(5,200,button_w,button_h);
redo_last_rect.rect(180,200,button_w,button_h);
deadzones_rect.rect(355,200,button_w,button_h);
done_calib_rect.rect(530,200,button_w,button_h);

get_current_cal_rect.rect(5,30,button_w,button_h);
send_calib_rect.rect(180,30,button_w,button_h);
save_calib_rect.rect(355,30,button_w,button_h);
finished_calib_rect.rect(530,30,button_w,button_h);

toggle_L_trigger_rect.rect(55,235,button_w,button_h);

const altx0 = 70;
const alty0 = 350;
left_trig_base_path.moveTo(altx0,alty0);
left_trig_base_path.arc(altx0,alty0-15,15,Math.PI/2,3*Math.PI/2,false);
left_trig_base_path.lineTo(altx0+130,alty0-30);
left_trig_base_path.arc(altx0+130,alty0-15,15,3*Math.PI/2,Math.PI/2,false);
left_trig_base_path.lineTo(altx0,alty0);

toggle_R_trigger_rect.rect(435,235,button_w,button_h);

const artx0 = 450;
const arty0 = 350;
right_trig_base_path.moveTo(artx0,arty0);
right_trig_base_path.arc(artx0,arty0-15,15,Math.PI/2,3*Math.PI/2,false);
right_trig_base_path.lineTo(artx0+130,arty0-30);
right_trig_base_path.arc(artx0+130,arty0-15,15,3*Math.PI/2,Math.PI/2,false);
right_trig_base_path.lineTo(artx0,arty0);

toggle_stick_raw_rect.rect(240,540,button_w,button_h);

var analog_stick_flag = 0;
var c_stick_flag = 0;

var store_val_flag = 0;
var done_calib_flag = 0;
var redo_last_store_flag = 0;
var get_current_cal_flag = 0;
var send_calib_flag = 0;
var save_calib_flag = 0;
var finished_calib_flag = 0;
var deadzones_flag = 0;
var trigger_flag = 0;
var toggle_triggers_flag = 0;
var toggle_stick_raw_flag = 0;


var display_msg = "Message";

const button_light_color = `rgb(192,192,192)`;
const button_dark_color = `rgb(150,150,150)`;

var store_val_colour_flag = 0;
var done_calib_colour_flag = 0;
var redo_last_store_colour_flag = 0;
var get_current_cal_colour_flag = 0;
var send_calib_colour_flag = 0;
var save_calib_colour_flag = 0;
var finished_calib_colour_flag = 0;
var deadzones_colour_flag = 0;
var toggle_L_trigger_colour_flag = 0;
var toggle_R_trigger_colour_flag = 0;
var L_Trigger_on = 1;
var R_Trigger_on = 1;
var toggle_stick_raw_colour_flag = 0;
var stick_raw_on_flag = 0;

function draw() {
    
    const canvas = document.getElementById("canvas");
    if (canvas.getContext) {
        const ctx = canvas.getContext("2d");
        ctx.canvas.width  = window.innerWidth;
        ctx.canvas.height = window.innerHeight;
        ctx.canvas.style.zIndex = "0";

        ctx.clearRect(0, 0, canvas.width, canvas.height); // clear the old drawings from the screen so can animate the sticks etc...
  
        if(analog_stick_flag){
            // making the analog stick gates
            var R = 60;
            var oX = 100;
            var oY = 470;

            ctx.lineWidth = 13;
            ctx.strokeStyle = `rgb(255,255,255)`;
            if(storageCounter >= 9 && store_val_flag){
                ctx.strokeStyle = `rgb(255,255,255,0.5)`;
            }
            

            drawOctogon(ctx,R,oX,oY);
            pop_analog_stick_bound_rect(R,oX,oY);

            if(store_val_flag){
                // making analog stick circle
                ctx.lineWidth = 5;
                ctx.beginPath();
                analog_stick_X = oX+R+R*analog_stick_cal_X_shifts[storageCounter];
                analog_stick_Y = oY+R*analog_stick_cal_Y_shifts[storageCounter];
                ctx.arc(analog_stick_X, analog_stick_Y, R-15, 0, Math.PI * 2, true); // Outer circle
                ctx.stroke();
                drawReadingText(ctx,"X = " + currentAX.toString(),oX+R-25,oY+R+40);
                drawReadingText(ctx,"Y = " + currentAY.toString(),oX+R-25,oY+R+70);
            }
            else{
                ctx.lineWidth = 5;
                ctx.beginPath();
                analog_stick_X = oX+R+R*(-1/2+mapStickVals(Curr_AX_Cal_Vals, currentAX,0,AnalogStickXDeadzone)/255);
                analog_stick_Y = oY+R*(1/2-mapStickVals(Curr_AY_Cal_Vals, currentAY,1,AnalogStickYDeadzone)/255);
                ctx.arc(analog_stick_X, analog_stick_Y, R-15, 0, Math.PI * 2, true); // Outer circle
                ctx.stroke();
                if(stick_raw_on_flag){
                    drawReadingText(ctx,"X = " + currentAX.toString(),oX+R-25,oY+R+40);
                    drawReadingText(ctx,"Y = " + currentAY.toString(),oX+R-25,oY+R+70);
                }
                else{
                    drawReadingText(ctx,"X = " + Math.round(mapStickVals(Curr_AX_Cal_Vals, currentAX,0,AnalogStickXDeadzone)).toString(),oX+R-25,oY+R+40);
                    drawReadingText(ctx,"Y = " + Math.round(mapStickVals(Curr_AY_Cal_Vals, currentAY,1,AnalogStickYDeadzone)).toString(),oX+R-25,oY+R+70);
                }
            }
        }

        if(c_stick_flag){
            // making the c-stick gates
            var R = 50;
            var oX = 400;
            var oY = 480;

            ctx.lineWidth = 13;
            ctx.strokeStyle = `rgb(250,245,6)`;
            if(storageCounter < 9 && store_val_flag){
                ctx.strokeStyle = `rgb(250,245,6,0.5)`;
            }

            drawOctogon(ctx,R,oX,oY);
            pop_c_stick_bound_rect(R,oX,oY);

            if(store_val_flag){
                // making c-stick circle
                ctx.lineWidth = 5;
                ctx.beginPath();
                c_stick_X = oX+R+R*c_stick_cal_X_shifts[storageCounter];
                c_stick_Y = oY+R*c_stick_cal_Y_shifts[storageCounter];
                ctx.arc(c_stick_X, c_stick_Y, R-20, 0, Math.PI * 2, true); // Outer circle
                ctx.stroke();
                drawReadingText(ctx,"X = " + currentCX.toString(),oX+R-25,oY+R+40);
                drawReadingText(ctx,"Y = " + currentCY.toString(),oX+R-25,oY+R+70);
            }
            else{
                ctx.lineWidth = 5;
                ctx.beginPath();
                c_stick_X = oX+R+R*(-1/2+mapStickVals(Curr_CX_Cal_Vals, currentCX,0, CStickXDeadzone)/255);
                c_stick_Y = oY+R*(1/2-mapStickVals(Curr_CY_Cal_Vals, currentCY,1, CStickYDeadzone)/255);
                ctx.arc(c_stick_X, c_stick_Y, R-20, 0, Math.PI * 2, true); // Outer circle
                ctx.stroke();
                if(stick_raw_on_flag){
                    drawReadingText(ctx,"X = " + currentCX.toString(),oX+R-25,oY+R+40);
                    drawReadingText(ctx,"Y = " + currentCY.toString(),oX+R-25,oY+R+70);
                }
                else{
                    drawReadingText(ctx,"X = " + Math.round(mapStickVals(Curr_CX_Cal_Vals, currentCX,0, CStickXDeadzone)).toString(),oX+R-25,oY+R+40);
                    drawReadingText(ctx,"Y = " + Math.round(mapStickVals(Curr_CY_Cal_Vals, currentCY,1, CStickYDeadzone)).toString(),oX+R-25,oY+R+70);
                }
            }
        }

        if(trigger_flag){
            ctx.fillStyle = `rgb(0,0,0)`;
            ctx.strokeStyle = `rgb(255,255,255)`;
            ctx.stroke(left_trig_base_path); // draw left trigger
            ctx.fill(left_trig_base_path);
            ctx.stroke(right_trig_base_path); // draw right trigger
            ctx.fill(right_trig_base_path);

            LT_pos = currentLT;
            RT_pos = currentRT;
            
            ctx.fillStyle = `rgb(255,255,255)`;
            ctx.beginPath();
            ctx.arc(altx0+2+LT_pos/2, alty0-15, 10, 0, Math.PI * 2, true);
            ctx.stroke();
            drawReadingText(ctx,"Left Trigger = "+LT_pos.toString(),altx0-5,alty0+30);

            ctx.beginPath();
            ctx.arc(artx0+2+RT_pos/2, arty0-15, 10, 0, Math.PI * 2, true);
            ctx.stroke();
            drawReadingText(ctx,"Right Trigger = "+RT_pos.toString(),artx0-5,arty0+30);

        }

        // other buttons like store, redo, done, etc...
        if(store_val_flag){
            drawButton(ctx,store_value_rect,store_val_colour_flag);
            if(storageCounter == numCalPoints*2){
                drawText(ctx,"Done",50,239);
            }
            else{
                drawText(ctx,"Store Value", 10, 239);
                // drawText(ctx,"Store", 50, 225);
                // drawText(ctx,"Value", 50, 255);
            }
            if(storageCounter == 0 || storageCounter == 9){
                drawText(ctx,Analog_Instructions[1], 10, 140); // some instructions
                drawText(ctx,Analog_Instructions[2], 10, 170); // some instructions
            }
            else{
                drawText(ctx,Analog_Instructions[3], 10, 140); // some instructions
                drawText(ctx,Analog_Instructions[4], 10, 170); // some instructions
            }
        }
        else{
            if(send_calib_flag){
                drawText(ctx,Analog_Instructions[0], 10, 150); // Some instructions
            }
        }
        if(done_calib_flag){
            drawButton(ctx,done_calib_rect,done_calib_colour_flag);

            drawText(ctx,"Exit", 575, 239);
        }
        if(redo_last_store_flag){
            drawButton(ctx,redo_last_rect,redo_last_store_colour_flag);
            drawText(ctx,"Redo",225,239);
        }
        if(get_current_cal_flag == 1){
            drawButton(ctx,get_current_cal_rect,get_current_cal_colour_flag);
            drawText(ctx,"Read",50,55);
            drawText(ctx,"Calibration",15,85);
        }
        if(get_current_cal_flag == 2){
            drawButton(ctx,get_current_cal_rect,get_current_cal_colour_flag);
            drawText(ctx,"Read",50,55);
            drawText(ctx,"Analog",40,85);
        }
        if(send_calib_flag){
            drawButton(ctx,send_calib_rect,send_calib_colour_flag);
            drawText(ctx,"Send",225,55);
            drawText(ctx,"Calibration",190,85);
        }
        if(save_calib_flag){
            drawButton(ctx,save_calib_rect,save_calib_colour_flag);
            drawText(ctx,"Save",402,55);
            drawText(ctx,"Calibration",365,85);
        }
        if(deadzones_flag){
            drawButton(ctx,deadzones_rect,deadzones_colour_flag);
            drawText(ctx,"Deadzones",360,239);
        }
        if(toggle_triggers_flag){
            drawButton(ctx,toggle_L_trigger_rect,toggle_L_trigger_colour_flag);
            drawText(ctx,"Toggle",60+30,274);
            if(L_Trigger_on){
                drawText(ctx,"ON",250,350); 
            }
            else{
                drawText(ctx,"OFF",250,350); 
            }
        }
        if(toggle_triggers_flag){
            drawButton(ctx,toggle_R_trigger_rect,toggle_R_trigger_colour_flag);
            drawText(ctx,"Toggle",440+30,274);
            if(R_Trigger_on){
                drawText(ctx,"ON",600,350); 
            }
            else{
                drawText(ctx,"OFF",600,350); 
            }
        }
        if(toggle_stick_raw_flag){
            drawButton(ctx,toggle_stick_raw_rect,toggle_stick_raw_colour_flag);
            if(stick_raw_on_flag){
                drawText(ctx,"Mapped",265,579);
            }
            else{
                drawText(ctx,"Raw",290,579);
            }
        }
        if(finished_calib_flag){
            drawButton(ctx,finished_calib_rect,finished_calib_colour_flag);
            drawText(ctx,"Finished",555,70);
        }
        else{
            clearInterval(drawInterval);
        }
        // drawText(ctx,display_msg,30,20);
    }
  }



// draws a filled octagon on canvas ctx, with "Radius" R, leftmost vertex at (oX,oY)
function drawOctogon(ctx,R,oX,oY){
    ctx.beginPath();
    ctx.moveTo(oX, oY);
    ctx.lineTo(oX+R-(Math.cos(Math.PI/4)*R),oY-(Math.sin(Math.PI/4)*R));
    ctx.lineTo(oX+R, oY-R);
    ctx.lineTo(oX+R+(Math.cos(Math.PI/4)*R),oY-(Math.sin(Math.PI/4)*R));
    ctx.lineTo(oX+2*R, oY);
    ctx.lineTo(oX+R+(Math.cos(Math.PI/4)*R),oY+(Math.sin(Math.PI/4)*R));
    ctx.lineTo(oX+R, oY+R);
    ctx.lineTo(oX+R-(Math.cos(Math.PI/4)*R),oY+(Math.sin(Math.PI/4)*R));
    ctx.closePath();
    ctx.stroke();
    ctx.fill();
}

function pop_analog_stick_bound_rect(R,oX,oY){
    analog_stick_bounding_rect.moveTo(oX,oY+R);
    analog_stick_bounding_rect.lineTo(oX,oY-R);
    analog_stick_bounding_rect.lineTo(oX+2*R,oY-R);
    analog_stick_bounding_rect.lineTo(oX+2*R,oY+R);
    analog_stick_bounding_rect.closePath();
}

function pop_c_stick_bound_rect(R,oX,oY){
    c_stick_bounding_rect.moveTo(oX,oY+R);
    c_stick_bounding_rect.lineTo(oX,oY-R);
    c_stick_bounding_rect.lineTo(oX+2*R,oY-R);
    c_stick_bounding_rect.lineTo(oX+2*R,oY+R);
    c_stick_bounding_rect.closePath();
}

function inter(){
    analog_stick_flag = 1;
    c_stick_flag = 1;
    get_current_cal_flag = 1;
    send_calib_flag = 1;
    save_calib_flag = 1;
    // deadzones_flag = 1; // ADD THIS BACK IF YOU WANT DEADZONE CALIBRATION
    finished_calib_flag = 1;
    trigger_flag = 1;
    toggle_triggers_flag = 1;
    toggle_stick_raw_flag = 1;
    drawInterval = setInterval(draw, 10); // calls draw every 10 ms
}

function drawButton(ctx,rect,flag = 0){
    ctx.fillStyle = button_light_color;
    if(flag == 1){
        ctx.fillStyle = button_dark_color;
    }
    ctx.strokeStyle = `rgb(0,0,0)`;
    ctx.stroke(rect);
    ctx.fill(rect);
}

function drawText(ctx,text,x,y){
    ctx.fillStyle = `rgb(0,0,0)`;
    ctx.font = "30px sans-serif";
    ctx.fillText(text, x, y)
}

function drawReadingText(ctx,text,x,y){
    ctx.fillStyle = `rgb(0,0,0)`;
    ctx.font = "20px sans-serif";
    ctx.fillText(text, x, y)
}





 function bisect(a, val, size){
    var lo = 0;
    var hi = size;
    var mid;

    while (lo < hi) {
        mid = Math.trunc((lo + hi) / 2);
        
        if (val < a[mid]) {
            hi = mid;
        }
        else {
            lo = mid + 1;
        }
    }
    return lo;
}




/**
 * @brief Use all of octagon notches to calibrate  
 * 
 * @note assumes calArray is sorted

*/
function mapStickVals(calArray, val, XorY, dead, printFlag=0){
    var mapped_val = 127;
    var numCalibPoints = 7;

    if (XorY == 0){
        // int sorted_cals[7] = {west, northWest,southWest, neutch, northEast, southEast, east};
        var west = calArray[0];
        var east = calArray[6];
        // var flipped_magnet_dir = (west<east) ? false : true;
        var flipped_magnet_dir = true;
        if(west<east){
            flipped_magnet_dir = false;
        }
    }
    else{
        // int sorted_cals[7] = {south, southWest,southEast, neutch, northEast, northWest, north};
        var south = calArray[0];
        var north = calArray[6];
        // var flipped_magnet_dir = (south < north) ? false : true;
        var flipped_magnet_dir = true;
        if(south<north){
            flipped_magnet_dir = false;
        }
    }
    
    var sortedCalArray = calArray.slice();
    sortedCalArray.sort(function(a, b){return a - b});

    // TODO: Find which section val is in
    var index = bisect(sortedCalArray, val, numCalibPoints);
    if(printFlag){
        console.log("index = " + index);
        console.log(sortedCalArray);
        console.log("val = " + val);
    }

    if (index == 0){
        mapped_val = 0;
    }
    else if (index == 1){
        var a = 0;
        var b = 37;

        var min_val = sortedCalArray[0];
        var max_val = sortedCalArray[1];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == 2){
        mapped_val = 37;
    }
    else if (index == 3){
        var a = 37;
        var b = 127;

        var min_val = sortedCalArray[2];
        var max_val = sortedCalArray[3];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == 4){
        var a = 127;
        var b = 218;

        var min_val = sortedCalArray[3];
        var max_val = sortedCalArray[4];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == 5){
        mapped_val = 218;
    }
    else if (index == 6){
        var a = 218;
        var b = 255;

        var min_val = sortedCalArray[5];
        var max_val = sortedCalArray[6];
        mapped_val = (a + (val - min_val)*(b-a) / (max_val - min_val));
    }
    else if (index == numCalibPoints){
        mapped_val = 255;
    }
    
    if(mapped_val >= dead.low && mapped_val <= dead.high){
        mapped_val = dead.value;
    }

    if(flipped_magnet_dir == true){
        if (mapped_val == 127){
            return 127
        }
        return 255-mapped_val;
    }

    
    return mapped_val;
}
