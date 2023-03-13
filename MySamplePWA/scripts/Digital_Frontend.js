
var digiDrawInterval;

const Digital_Instructions = [
    "Click 2 buttons in sequence to swap them",
    "Click the same button twice to toggle it"
];

const A_Button_Path = new Path2D();
const B_Button_Path = new Path2D();
const S_Button_Path = new Path2D();

const DU_Button_Path = new Path2D();
const DR_Button_Path = new Path2D();
const DD_Button_Path = new Path2D();
const DL_Button_Path = new Path2D();
const DC_Button_Path = new Path2D();
const DUT_Path = new Path2D();
const DRT_Path = new Path2D();
const DDT_Path = new Path2D();
const DLT_Path = new Path2D();
const DCC_Path = new Path2D();

const L_Button_Path = new Path2D();
const R_Button_Path = new Path2D();

const X_Button_Path = new Path2D();
const Y_Button_Path = new Path2D();
const Z_Button_Path = new Path2D();

const Read_Mapping_Button_Path = new Path2D();
const Send_Mapping_Button_Path = new Path2D();
const Save_Mapping_Button_Path = new Path2D();
const Done_Mapping_Button_Path = new Path2D();

const Redo_Last_Button_Path = new Path2D();
const Reset_Default_Mapping_Path = new Path2D();

const A_Tag_Path = new Path2D();
const B_Tag_Path = new Path2D();
const X_Tag_Path = new Path2D();
const Y_Tag_Path = new Path2D();
const Z_Tag_Path = new Path2D();
const L_Tag_Path = new Path2D();
const R_Tag_Path = new Path2D();
const S_Tag_Path = new Path2D();
const DU_Tag_Path = new Path2D();
const DR_Tag_Path = new Path2D();
const DL_Tag_Path = new Path2D();
const DD_Tag_Path = new Path2D();

const ax0 = 435;
const ay0 = 350 + 200;
A_Button_Path.arc(ax0,ay0,40,0,Math.PI*2,true);

const bx0 = 370;
const by0 = ay0+60;
B_Button_Path.arc(bx0,by0,20,0,Math.PI*2,true);

const sx0 = 280;
const sy0 = ay0;
S_Button_Path.arc(sx0,sy0,10,0,Math.PI*2,true);

const dux0 = 120;
const duy0 = by0;
DU_Button_Path.rect(dux0,duy0,20,20);

const drx0 = dux0+20;
const dry0 = duy0+20;
DR_Button_Path.rect(drx0,dry0,20,20);

const ddx0 = dux0;
const ddy0 = duy0+40;
DD_Button_Path.rect(ddx0,ddy0,20,20);

const dlx0 = dux0-20;
const dly0 = duy0+20;
DL_Button_Path.rect(dlx0,dly0,20,20);

const dcx0 = dux0;
const dcy0 = duy0+20;
DC_Button_Path.rect(dcx0,dcy0,20,20);

DUT_Path.moveTo(dux0+2,duy0+15);
DUT_Path.lineTo(dux0+9,duy0);
DUT_Path.lineTo(dux0+17,duy0+15);

DRT_Path.moveTo(drx0+3,dry0+2);
DRT_Path.lineTo(drx0+18,dry0+9);
DRT_Path.lineTo(drx0+3,dry0+16);

DDT_Path.moveTo(ddx0+2,ddy0+2);
DDT_Path.lineTo(ddx0+9,ddy0+17);
DDT_Path.lineTo(ddx0+17,ddy0+2);

DLT_Path.moveTo(dlx0+17,dly0+2);
DLT_Path.lineTo(dlx0+2,dly0+9);
DLT_Path.lineTo(dlx0+17,dly0+16);

DCC_Path.arc(dcx0+10,dcy0+10,8,0,Math.PI*2,true);

const xx0 = 490;
const xy0 = ay0-15;
X_Button_Path.moveTo(xx0,xy0);
X_Button_Path.arc(xx0+15,xy0,15,Math.PI,0,false);
X_Button_Path.lineTo(xx0+30,xy0+50);
X_Button_Path.arc(xx0+15,xy0+50,15,0,Math.PI,false);
X_Button_Path.lineTo(xx0,xy0);

const yx0 = 420;
const yy0 = ay0-55;
Y_Button_Path.moveTo(yx0,yy0);
Y_Button_Path.arc(yx0,yy0-15,15,Math.PI/2,3*Math.PI/2,false);
Y_Button_Path.lineTo(yx0+50,yy0-30);
Y_Button_Path.arc(yx0+50,yy0-15,15,3*Math.PI/2,Math.PI/2,false);
Y_Button_Path.lineTo(yx0,yy0);

const zx0 = 435;
const zy0 = ay0-130;
Z_Button_Path.rect(zx0,zy0,90,25,8);

const lx0 = 50;
const ly0 = ay0-150;
L_Button_Path.moveTo(lx0,ly0);
L_Button_Path.lineTo(lx0,ly0-70);
L_Button_Path.arc(lx0+30,ly0-70,30,Math.PI,0,false);
L_Button_Path.lineTo(lx0+60,ly0);
L_Button_Path.lineTo(lx0,ly0);

const rx0 = 450;
const ry0 = ly0;
R_Button_Path.moveTo(rx0,ry0);
R_Button_Path.lineTo(rx0,ry0-70);
R_Button_Path.arc(rx0+30,ry0-70,30,Math.PI,0,false);
R_Button_Path.lineTo(rx0+60,ry0);
R_Button_Path.lineTo(rx0,ry0);

const rmx0 = 5;
const rmy0 = 30;
Read_Mapping_Button_Path.rect(rmx0,rmy0,button_w,button_h);

const snmx0 = rmx0+175;
Send_Mapping_Button_Path.rect(snmx0,rmy0,button_w,button_h);

const svmx0 = snmx0+175;
Save_Mapping_Button_Path.rect(svmx0,rmy0,button_w,button_h);

const dmx0 = svmx0+175;
Done_Mapping_Button_Path.rect(dmx0,rmy0,button_w,button_h);

const rlbx0 = 200;
const rlby0 = 200;
Redo_Last_Button_Path.rect(rlbx0,rlby0,button_w,button_h);

const rdmx0 = 5;
const rdmy0 = 200;
Reset_Default_Mapping_Path.rect(rdmx0,rdmy0,button_w,button_h);

const atx0 = ax0+24;
const aty0 = ay0-35;
A_Tag_Path.moveTo(atx0,aty0);
populateTags(A_Tag_Path,atx0,aty0,1);

const btx0 = bx0;
const bty0 = by0+22;
B_Tag_Path.moveTo(btx0,bty0);
populateTags(B_Tag_Path,btx0,bty0,1);

const xtx0 = xx0+30;
const xty0 = xy0+40;
X_Tag_Path.moveTo(xtx0,xty0);
populateTags(X_Tag_Path,xtx0,xty0,1);

const ytx0 = yx0+65;
const yty0 = yy0-15;
Y_Tag_Path.moveTo(ytx0,yty0);
populateTags(Y_Tag_Path,ytx0,yty0,1);

const ztx0 = zx0+90;
const zty0 = zy0+10;
Z_Tag_Path.moveTo(ztx0,zty0);
populateTags(Z_Tag_Path,ztx0,zty0,1);

const ltx0 = lx0+60;
const lty0 = ly0-40;
L_Tag_Path.moveTo(ltx0,lty0);
populateTags(L_Tag_Path,ltx0,lty0,1);

const rtx0 = rx0+60;
const rty0 = ry0-40;
R_Tag_Path.moveTo(rtx0,rty0);
populateTags(R_Tag_Path,rtx0,rty0,1);

const stx0 = sx0-10;
const sty0 = sy0;
S_Tag_Path.moveTo(stx0,sty0);
populateTags(S_Tag_Path,stx0,sty0,-1);

const dutx0 = dux0;
const duty0 = duy0;
DU_Tag_Path.moveTo(dutx0,duty0);
populateTags(DU_Tag_Path,dutx0,duty0,1);

const drtx0 = drx0;
const drty0 = dry0+20;
DR_Tag_Path.moveTo(drtx0,drty0);
populateTags(DR_Tag_Path,drtx0,drty0,1);

const ddtx0 = ddx0;
const ddty0 = ddy0+16;
DD_Tag_Path.moveTo(ddtx0,ddty0);
populateTags(DD_Tag_Path,ddtx0,ddty0,-0.28);

const dltx0 = dlx0;
const dlty0 = dly0;
DL_Tag_Path.moveTo(dltx0,dlty0);
populateTags(DL_Tag_Path,dltx0,dlty0,-0.28);


var A_Button_flag = 0;
var B_Button_flag = 0;
var S_Button_flag = 0;
var D_Button_flag = 0;
var X_Button_flag = 0;
var Y_Button_flag = 0;
var Z_Button_flag = 0;
var L_Button_flag = 0;
var R_Button_flag = 0;
var Draw_Tags_Flag = 0;

var Read_Mapping_Button_Flag = 0;
var Send_Mapping_Button_Flag = 0;
var Save_Mapping_Button_Flag = 0;
var Done_Mapping_Button_Flag = 0;

var Redo_Last_Button_Flag = 0;
var Reset_Default_Mapping_Flag = 0;

var Read_Mapping_Button_Colour_Flag = 0;
var Send_Mapping_Button_Colour_Flag = 0;
var Save_Mapping_Button_Colour_Flag = 0;
var Done_Mapping_Button_Colour_Flag = 0;
var Redo_Last_Button_Colour_Flag = 0;
var Reset_Default_Mapping_Colour_Flag = 0;

function drawDigital(){
    const canvas = document.getElementById("canvas");
    if (canvas.getContext) {
        const ctx = canvas.getContext("2d");
        ctx.canvas.width  = window.innerWidth;
        ctx.canvas.height = window.innerHeight;
        ctx.canvas.style.zIndex = "0";

        ctx.clearRect(0, 0, canvas.width, canvas.height);

        // Start Generate Gamecube Buttons
        if(A_Button_flag){
            ctx.strokeStyle = `rgb(68,213,208)`;
            ctx.fillStyle = `rgb(68,213,208)`;
            ctx.stroke(A_Button_Path);
            ctx.fill(A_Button_Path);
            ctx.strokeStyle = `rgb(53,179,175)`;
            ctx.fillStyle = `rgb(53,179,175)`;
            ctx.font = "45px sans-serif";
            ctx.fillText("A", ax0-15, ay0+15);
        }

        if(B_Button_flag){
            ctx.strokeStyle = `rgb(249,21,48)`;
            ctx.fillStyle = `rgb(249,21,48)`;
            ctx.stroke(B_Button_Path);
            ctx.fill(B_Button_Path);
            ctx.strokeStyle = `rgb(174,4,26)`;
            ctx.fillStyle = `rgb(174,4,26)`;
            ctx.font = "22px sans-serif";
            ctx.fillText("B", bx0-7, by0+8);
        }
        if(S_Button_flag){
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.stroke(S_Button_Path);
            ctx.fill(S_Button_Path);
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.font = "10px sans-serif";
            ctx.fillText("START/PAUSE", sx0-33, sy0-15);
        }

        if(D_Button_flag){
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.stroke(DU_Button_Path);
            ctx.fill(DU_Button_Path);
            ctx.stroke(DR_Button_Path);
            ctx.fill(DR_Button_Path);
            ctx.stroke(DD_Button_Path);
            ctx.fill(DD_Button_Path);
            ctx.stroke(DL_Button_Path);
            ctx.fill(DL_Button_Path);
            ctx.stroke(DC_Button_Path);
            ctx.fill(DC_Button_Path);

            ctx.fillStyle = `rgb(203,200,207)`;
            ctx.fill(DUT_Path);
            ctx.fill(DRT_Path);
            ctx.fill(DDT_Path);
            ctx.fill(DLT_Path);
            ctx.fill(DCC_Path);
        }

        if(X_Button_flag){
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.stroke(X_Button_Path);
            ctx.fill(X_Button_Path);
            ctx.fillStyle = `rgb(203,200,207)`;
            ctx.font = "20px sans-serif";
            ctx.fillText("X", xx0+8, xy0+32);
        }

        if(Y_Button_flag){
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.stroke(Y_Button_Path);
            ctx.fill(Y_Button_Path);
            ctx.fillStyle = `rgb(203,200,207)`;
            ctx.font = "20px sans-serif";
            ctx.fillText("Y", yx0+20, yy0-5);
        }

        if(Z_Button_flag){
            ctx.strokeStyle = `rgb(67,94,225)`;
            ctx.fillStyle = `rgb(67,94,225)`;
            ctx.stroke(Z_Button_Path);
            ctx.fill(Z_Button_Path);
            ctx.fillStyle = `rgb(33,64,207)`;
            ctx.font = "20px sans-serif";
            ctx.fillText("Z", zx0+40, zy0+20);
        }

        if(L_Button_flag){
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.stroke(L_Button_Path);
            ctx.fill(L_Button_Path);
            ctx.fillStyle = `rgb(203,200,207)`;
            ctx.font = "30px sans-serif";
            ctx.fillText("L", lx0+23, ly0-37);
        }
        if(R_Button_flag){
            ctx.strokeStyle = `rgb(240,237,246)`;
            ctx.fillStyle = `rgb(240,237,246)`;
            ctx.stroke(R_Button_Path);
            ctx.fill(R_Button_Path);
            ctx.fillStyle = `rgb(203,200,207)`;
            ctx.font = "30px sans-serif";
            ctx.fillText("R", rx0+18, ry0-37);
        }
        if(Draw_Tags_Flag){
            drawTags(ctx);
        }
        // End Generate Gamecube Buttons

        // Start Generate IO Buttons
        if(Read_Mapping_Button_Flag){
            drawButton(ctx,Read_Mapping_Button_Path,Read_Mapping_Button_Colour_Flag);
            drawText(ctx,"Read", rmx0+45, rmy0+25);
            drawText(ctx,"Mapping", rmx0+25, rmy0+55);

            drawText(ctx,Digital_Instructions[0], 10, 140); // some instructions
            drawText(ctx,Digital_Instructions[1], 10, 170); // some instructions
        }
        if(Send_Mapping_Button_Flag){
            drawButton(ctx,Send_Mapping_Button_Path,Send_Mapping_Button_Colour_Flag);
            drawText(ctx,"Send", snmx0+45, rmy0+25);
            drawText(ctx,"Mapping", snmx0+25, rmy0+55);
        }
        if(Save_Mapping_Button_Flag){
            drawButton(ctx,Save_Mapping_Button_Path,Save_Mapping_Button_Colour_Flag);
            drawText(ctx,"Save", svmx0+45, rmy0+25);
            drawText(ctx,"Mapping", svmx0+25, rmy0+55);
        }
        if(Redo_Last_Button_Flag){
            drawButton(ctx,Redo_Last_Button_Path,Redo_Last_Button_Colour_Flag);
            drawText(ctx,"Redo Last", rlbx0+10, rlby0+40);
        }
        drawCurrButtonText(ctx,first_clicked);
        if(Reset_Default_Mapping_Flag){
            drawButton(ctx,Reset_Default_Mapping_Path,Reset_Default_Mapping_Colour_Flag);
            drawText(ctx,"Reset", rdmx0+40, rdmy0+40);
        }
        if(Done_Mapping_Button_Flag){
            drawButton(ctx,Done_Mapping_Button_Path,Done_Mapping_Button_Colour_Flag);
            drawText(ctx,"Done", dmx0+45, rmy0+25);
            drawText(ctx,"Mapping", dmx0+25, rmy0+55);
        }
        else{
            clearInterval(digiDrawInterval);
        }

    }
}

function DigitalInter(){
    A_Button_flag = 1;
    B_Button_flag = 1;
    S_Button_flag = 1;
    D_Button_flag = 1;
    X_Button_flag = 1;
    Y_Button_flag = 1;
    Z_Button_flag = 1;
    L_Button_flag = 1;
    R_Button_flag = 1;

    Draw_Tags_Flag = 1;

    Read_Mapping_Button_Flag = 1;
    Send_Mapping_Button_Flag = 1;
    Save_Mapping_Button_Flag = 1;
    Done_Mapping_Button_Flag = 1;
    Reset_Default_Mapping_Flag = 1;
    digiDrawInterval = setInterval(drawDigital, 10); // calls draw every 10 ms
}

function drawCurrButtonText(ctx,text){
    if(text=="S"){
        text = "Start";
    }
    else if(text == "u"){
        text = "D-UP";
    }
    else if(text == "r"){
        text = "D-RIGHT";
    }
    else if(text == "d"){
        text = "D-DOWN";
    }
    else if(text == "l"){
        text = "D-LEFT";
    }
    ctx.fillStyle = `rgb(0,0,0)`;
    ctx.font = "30px sans-serif";
    if(num_clicked == 1){
        ctx.fillText("Currently Selected: " + text, rlbx0+175, rdmy0+40);
    }
    else{
        if(first_clicked != "" && second_clicked != ""){
            switch(what_button_msg_flag){
                case(1):
                    ctx.fillText("Toggled: " + first_clicked, rlbx0+175, rdmy0+40);
                    break;
                case(2):
                    ctx.fillText("Swapped: " + first_clicked + " & " + second_clicked, rlbx0+175, rdmy0+40);
                    break;
            }
        }
    }
}

function populateTags(tagObj,x,y,direc){
    const x_tagline_len = 100*direc;
    const box_size_x = 70*direc/Math.abs(direc);
    const box_size_y = 30*direc/Math.abs(direc);
    tagObj.lineTo(x+x_tagline_len,y);
    tagObj.lineTo(x+x_tagline_len,y-box_size_y/2);
    tagObj.lineTo(x+x_tagline_len+box_size_x,y-box_size_y/2);
    tagObj.lineTo(x+x_tagline_len+box_size_x,y+box_size_y/2);
    tagObj.lineTo(x+x_tagline_len,y+box_size_y/2);
    tagObj.lineTo(x+x_tagline_len,y);
}

function drawTags(ctx){
    ctx.strokeStyle = `rgb(0,0,0)`;
    ctx.stroke(A_Tag_Path);
    ctx.stroke(B_Tag_Path);
    ctx.stroke(X_Tag_Path);
    ctx.stroke(Y_Tag_Path);
    ctx.stroke(Z_Tag_Path);
    ctx.stroke(S_Tag_Path);
    ctx.stroke(L_Tag_Path);
    ctx.stroke(R_Tag_Path);
    ctx.stroke(DU_Tag_Path);
    ctx.stroke(DR_Tag_Path);
    ctx.stroke(DD_Tag_Path);
    ctx.stroke(DL_Tag_Path);

    drawButtonMapText(ctx,Digital_Button_Map.get("A")+", "+onOrOff(Toggle_Map.get("A")),atx0+103,aty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("B")+", "+onOrOff(Toggle_Map.get("B")),btx0+103,bty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("X")+", "+onOrOff(Toggle_Map.get("X")),xtx0+103,xty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("Y")+", "+onOrOff(Toggle_Map.get("Y")),ytx0+103,yty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("Z")+", "+onOrOff(Toggle_Map.get("Z")),ztx0+103,zty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("S")+", "+onOrOff(Toggle_Map.get("S")),stx0-168,sty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("L")+", "+onOrOff(Toggle_Map.get("L")),ltx0+103,lty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("R")+", "+onOrOff(Toggle_Map.get("R")),rtx0+103,rty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("u")+", "+onOrOff(Toggle_Map.get("u")),dutx0+103,duty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("r")+", "+onOrOff(Toggle_Map.get("r")),drtx0+103,drty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("d")+", "+onOrOff(Toggle_Map.get("d")),ddtx0-92,ddty0+7);
    drawButtonMapText(ctx,Digital_Button_Map.get("l")+", "+onOrOff(Toggle_Map.get("l")),dltx0-92,dlty0+7);
}

function drawButtonMapText(ctx,text,x,y){
    ctx.fillStyle = `rgb(0,0,0)`;
    ctx.font = "20px sans-serif";
    ctx.fillText(text, x, y)
}

function onOrOff(t){
    if(t == "Y"){
        return "ON";
    }
    else{
        if(t=="N"){
            return "OFF";
        }
        else{
            return "IDK"
        }
    }
}