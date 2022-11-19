#ifndef WEB_PAGE_H
#define WEB_PAGE_H
#define WEB_PAGE "<html><head><title>ClockClock 24 (replica)</title><link rel=icon href=\"data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 490 490'%3E%3Cstyle xmlns='http://www.w3.org/2000/svg'%3E %23fav %7B stroke: %23000; fill: %23000; %7D @media (prefers-color-scheme: dark) %7B %23fav %7B stroke: %23fff; fill: %23fff; %7D %7D %3C/style%3E%3Cg id='fav'%3E%3Cg%3E%3Cpath d='M233.004,0C104.224,0,0,104.212,0,233.004c0,128.781,104.212,233.004,233.004,233.004 c128.782,0,233.004-104.212,233.004-233.004C466.008,104.222,361.796,0,233.004,0z M244.484,242.659l-63.512,75.511 c-5.333,6.34-14.797,7.156-21.135,1.824c-6.34-5.333-7.157-14.795-1.824-21.135l59.991-71.325V58.028c0-8.284,6.716-15,15-15 s15,6.716,15,15v174.976h0C248.004,236.536,246.757,239.956,244.484,242.659z'/%3E%3C/g%3E%3C/g%3E%3C/svg%3E\"type=image/svg+xml><style>html{width:100%;height:100%}body{display:flex;flex-direction:column;align-items:center;justify-content:center;font-family:Tahoma,Helvetica,sans-serif;color:#fff;background-color:#212121;cursor:default;user-select:none;font-size:14px}#art{margin:80px 0}.title{font-size:16px;font-weight:700;margin-bottom:8px;text-align:center}.text{margin-bottom:8px;text-align:center}a{color:#8a8a8a;text-decoration:none}.half-digit{float:left}.clock{--small-hand:90deg;--large-hand:90deg;--animation-time:15s;width:8vw;height:8vw;border-radius:50%;box-shadow:inset 0 0 3px #fff;float:left;cursor:pointer;position:absolute}.clock:nth-of-type(n+1){clear:left}.clock-largeHand,.clock-smallHand{transform-origin:50px center;transition-timing-function:ease;transition:transform var(--animation-time)}.clock-smallHand{transform:rotateZ(var(--small-hand))}.clock-largeHand{transform:rotateZ(var(--large-hand))}.hidden .btn-clock{display:none}#external.hidden{display:none}.btn-clock{position:absolute;width:4vw;height:4vw;background-color:#212121dc;box-shadow:inset 0 0 3px #fff;font-size:.8vw;font-weight:700;display:flex;flex-direction:row;align-content:center;justify-content:center;align-items:center;cursor:pointer}.btn-clock:hover{background-color:#c60a0a}.btn-clock.btn-clock-tl{border-radius:4vw 0 0 0}.btn-clock.btn-clock-tr{margin-left:4vw;border-radius:0 4vw 0 0}.btn-clock.btn-clock-bl{margin-top:4vw;border-radius:0 0 0 4vw}.btn-clock.btn-clock-br{margin-left:4vw;margin-top:4vw;border-radius:0 0 4vw 0}.clock-box{width:8vw;height:8vw;margin:.2vw}.btn{width:72px;height:48px;margin:4px;margin-bottom:8px;font-size:14px;border-width:0;padding:0;background-color:transparent;color:#fff;box-shadow:inset 0 0 2px #dfdfdf;cursor:pointer;display:flex;flex-direction:column;align-items:center;justify-content:center;float:left}.btn:hover{background-color:#b4b4b44b}.btn.active{background-color:#dfdfdf;box-shadow:inset 0 0 0 2px #dfdfdf;color:#000}.btn.day-active{background-color:#dfdfdf;box-shadow:inset 0 0 0 2px #dfdfdf;color:#000;padding-bottom:8px;margin-bottom:0}.checkbox{width:44px;height:16px;box-shadow:inset 0 0 1px #000;float:left;margin:2px;margin-bottom:8px}.checkbox:hover{background-color:#b4b4b4}.checkbox.selected{background-color:#000}.hour-text{width:48px;height:16px;color:#212121dc;display:flex;flex-direction:column;align-items:center;justify-content:center;float:left;font-size:12px;margin-top:4px}.spacer{height:24px}.input-text{width:172px;height:48px;margin:4px;box-shadow:inset 0 0 2px #dfdfdf;background-color:transparent;border-color:transparent;color:#fff;padding-top:0;padding-bottom:0;padding-left:16px;padding-right:16px;font-size:16px;float:left;display:inline;white-space:nowrap;border-width:0}.input-text:focus-visible{outline:transparent}.input-text::selection{color:#000;background:#fff}.hours-box{background-color:#dfdfdf;display:flex;flex-direction:column;align-items:center;padding:16px}</style></head><body id=body><div class=spacer></div><h1>ClockClock 24 (replica)</h1><div id=art></div><div class=spacer></div><div class=title>Mode</div><div id=modes></div><div class=spacer></div><div class=title>Sleep Time</div><div id=days></div><div id=hours></div><div class=spacer></div><div class=title>Wireless Connection</div><div id=wifi><div class=btn style=width:150px id=con-0 onclick=selectConnection(0)>HOTSPOT</div><div class=btn style=width:150px id=con-1 onclick=selectConnection(1)>EXTERNAL</div></div><form id=external class=hidden onsubmit=return!1><input id=ssid class=input-text placeholder=SSID minlength=1 required> <input type=password id=password class=input-text placeholder=PASSWORD minlength=1 required> <button type=submit class=btn style=width:100px onclick=saveConnection()>SAVE</button></form><div class=spacer></div><div class=spacer></div><div class=text><p><small>Code</small><br><a href=http://www.vallasc.github.com/ >Giacomo Vallorani</a></p><p><small>Clock animation</small><br><a href=https://manu.ninja/ >Manuel Wieser</a></p><p><small>Design</small><br><a href=http://www.humanssince1982.com/ >Humans since 1982</a></p></div><script>let sleep=[Array(24).fill(0),Array(24).fill(0),Array(24).fill(0),Array(24).fill(0),Array(24).fill(0),Array(24).fill(0),Array(24).fill(0)],selectedMode=0,selectedClock=void 0,selectedDay=void 0,selectedConnection=void 0,ssid=\"\",password=\"\";function clock(e){return`<div id=\"clock-${e}\" class=\"clock-box hidden\"><svg class=\"clock clock--${e}\" width=\"100\" height=\"100\" viewBox=\"0 0 100 100\" onclick=\"selectClock(${e})\"><path class=\"clock-smallHand\" d=\"M50,47 C48.3431458,47 47,48.3431458 47,50 C47,51.6568542 48.3431458,53 50,53 L95,53 L95,47 L50,47 Z\" stroke=\"none\" fill=\"#FFF\" fill-rule=\"evenodd\"></path><path class=\"clock-largeHand\" d=\"M50,47 C48.3431458,47 47,48.3431458 47,50 C47,51.6568542 48.3431458,53 50,53 L100,53 L100,47 L50,47 Z\" stroke=\"none\" fill=\"#FFF\" fill-rule=\"evenodd\"></path></svg><div class=\"btn-clock btn-clock-tl\" onclick=\"adjustHand(${e}, 0, 1)\">H+</div><div class=\"btn-clock btn-clock-tr\" onclick=\"adjustHand(${e}, 1, 0)\">M+</div><div class=\"btn-clock btn-clock-bl\" onclick=\"adjustHand(${e}, 0, -1)\">H-</div><div class=\"btn-clock btn-clock-br\" onclick=\"adjustHand(${e}, -1, 0)\">M-</div></div>`}function genModes(){var e;let t=\"\",l=0;for(e of[\"LAZY\",\"FUN\",\"WAVES\",\"OFF\"])t+=`<div id=\"mode-${l}\" class=\"btn ${l===selectedMode?\"active\":\"\"}\" onclick=\"selectMode(${l++})\">${e}</div>`;document.getElementById(\"modes\").innerHTML=t}function genClocks(){let t='<div class=\"half-digit\">';for(let e=0;e<24;e++)e%3==0&&0!=e&&(t+='</div><div class=\"half-digit\">'),t+=clock(e);t+=\"</div>\",document.getElementById(\"art\").innerHTML=t}function genDays(){var e;let t=\"\",l=0;for(e of[\"MON\",\"TUE\",\"WED\",\"THU\",\"FRI\",\"SAT\",\"SUN\"])t+=`<div id=\"day-${l}\" class=\"btn\" onclick=\"selectDay(${l++})\">${e}</div>`;document.getElementById(\"days\").innerHTML=t}function genHours(t){let l='<div class=\"hours-box\"><div>';for(let e=0;e<=12;e++)l+=`<div class=\"hour-text\">${e}</div>`;l+=\"</div><div>\";for(let e=0;e<12;e++)l+=`<div id=\"hour-${e}\" class=\"checkbox ${1===sleep[t][e]?\"selected\":\"\"}\" onclick=\"selectHour(${e})\"></div>`;l+=\"</div><div>\";for(let e=12;e<=24;e++)l+=`<div class=\"hour-text\">${e%24}</div>`;l+=\"</div><div>\";for(let e=12;e<24;e++)l+=`<div id=\"hour-${e}\" class=\"checkbox ${1===sleep[t][e]?\"selected\":\"\"}\" onclick=\"selectHour(${e})\"></div>`;l+=\"</div></div>\",document.getElementById(\"hours\").innerHTML=l}function selectMode(e){(3===e?stopClock:startClock)(),void 0!==selectedMode&&document.getElementById(\"mode-\"+selectedMode).classList.remove(\"active\"),selectedMode=e,document.getElementById(\"mode-\"+selectedMode).classList.add(\"active\"),saveMode(selectedMode)}function selectDay(e){deselectDay(),selectedDay!==e&&(selectedDay=e,document.getElementById(\"day-\"+selectedDay).classList.add(\"day-active\"),genHours(e))}function deselectDay(){void 0!==selectedDay&&document.getElementById(\"day-\"+selectedDay).classList.remove(\"day-active\"),selectedDay=void 0,document.getElementById(\"hours\").innerHTML=\"\"}function selectHour(e){var t=sleep[parseInt(selectedDay)][e];sleep[parseInt(selectedDay)][e]=1===t?0:1,0===t?document.getElementById(\"hour-\"+e).classList.add(\"selected\"):document.getElementById(\"hour-\"+e).classList.remove(\"selected\"),saveSleepTime(selectedDay,sleep[parseInt(selectedDay)])}function deselectClock(){void 0!==selectedClock&&document.getElementById(\"clock-\"+selectedClock).classList.add(\"hidden\"),selectedClock=void 0}function selectClock(e){deselectClock(),selectedClock=e,document.getElementById(\"clock-\"+selectedClock).classList.remove(\"hidden\")}function selectConnection(e){void 0!==selectedConnection&&(document.getElementById(\"con-\"+selectedConnection).classList.remove(\"active\"),document.getElementById(\"external\").classList.add(\"hidden\")),0===e&&selectedConnection!==e&&saveConnection(),1===(selectedConnection=e)&&document.getElementById(\"external\").classList.remove(\"hidden\"),document.getElementById(\"con-\"+selectedConnection).classList.add(\"active\")}function sendDate(){var e=new Date,t=new FormData;t.append(\"h\",e.getHours().toString()),t.append(\"m\",e.getMinutes().toString()),t.append(\"s\",e.getSeconds().toString()),t.append(\"D\",e.getDate().toString()),t.append(\"M\",(e.getMonth()+1).toString()),t.append(\"Y\",e.getFullYear().toString()),t.append(\"timezone\",(e.getTimezoneOffset()/-60).toString()),fetch(\"/time\",{method:\"post\",body:t})}async function updateConfig(){var e=await(await fetch(\"/config\",{method:\"get\"})).json();sleep=e.sleep_time,void 0!==selectedDay&&genHours(selectedDay),selectMode(e.clock_mode),selectConnection(e.wireless_mode),document.getElementById(\"ssid\").value=ssid=e.ssid,document.getElementById(\"password\").value=password=e.password}function adjustHand(e,t,l){var o=new FormData;o.append(\"index\",e.toString()),o.append(\"m_amount\",t.toString()),o.append(\"h_amount\",l.toString()),fetch(\"/adjust\",{method:\"post\",body:o})}function saveMode(e){var t=new FormData;t.append(\"mode\",e.toString()),fetch(\"/mode\",{method:\"post\",body:t})}document.addEventListener(\"click\",function(e){\"body\"===e.target.id&&(deselectClock(),deselectDay())}),genClocks(),genModes(),genDays();let lastSleepDay=void 0,sleepTimeout=void 0;function saveSleepTime(e,l){lastSleepDay===e&&clearTimeout(sleepTimeout),lastSleepDay=e,sleepTimeout=setTimeout(()=>{var t=new FormData;t.append(\"day\",e.toString());for(let e=0;e<24;e++)t.append(\"h\"+e,l[e].toString());fetch(\"/sleep\",{method:\"post\",body:t})},1500)}async function saveConnection(){ssid=document.getElementById(\"ssid\").value,password=document.getElementById(\"password\").value;var e=new FormData;e.append(\"mode\",selectedConnection.toString()),e.append(\"ssid\",ssid),e.append(\"password\",password),await fetch(\"/connection\",{method:\"post\",body:e}),location.reload()}sendDate(),updateConfig();const digit_stop=[[270,270],[270,270],[270,270],[270,270],[270,270],[270,270]],digit_II=[[270,90],[270,90],[270,90],[270,90],[270,90],[270,90]],digits=[[[270,0],[270,90],[0,90],[270,180],[270,90],[180,90]],[[225,225],[225,225],[225,225],[270,270],[270,90],[90,90]],[[0,0],[270,0],[90,0],[180,270],[90,180],[180,180]],[[0,0],[0,0],[0,0],[180,270],[180,90],[180,90]],[[270,270],[90,0],[225,225],[270,270],[270,90],[90,90]],[[270,0],[90,0],[0,0],[180,180],[270,180],[90,180]],[[270,0],[270,90],[90,0],[180,180],[270,180],[90,180]],[[0,0],[225,225],[225,225],[270,180],[270,90],[90,90]],[[270,0],[90,0],[90,0],[270,180],[90,180],[90,180]],[[270,0],[0,90],[0,0],[270,180],[270,90],[90,180]]];let anim_state=Array(4).fill().map(()=>Array(6).fill().map(()=>[90,90])),current_state=Array(4).fill().map(()=>Array(6).fill().map(()=>[270,270]));function setHands(e,t,l,o){e=document.querySelector(\".clock--\"+e);e.style.setProperty(\"--small-hand\",t+\"deg\"),e.style.setProperty(\"--large-hand\",l+\"deg\"),e.style.setProperty(\"--animation-time\",o+\"s\")}function calcAngleCCW(e,t){t=(t-e)%360;return Math.abs(t<=0?-t:360-t)}function calcAngleCW(e,t){t=(t-e)%360;return Math.abs(t<0?360+t:t)}function setHalfDigit(t,l,o,s){for(let e=0;e<3;e++){var c=current_state[Math.floor(t/2)][e+t%2*3],n=anim_state[Math.floor(t/2)][e+t%2*3],d=calcAngleCW(c[0],l[e][0]),i=calcAngleCCW(c[0],l[e][0]),a=calcAngleCW(c[1],l[e][1]),r=calcAngleCCW(c[1],l[e][1]),v=o%3*360;o<=2?(c[0]=(c[0]-i)%360,c[0]=c[0]<0?c[0]+=360:c[0],n[0]=n[0]+i+v,c[1]=(c[1]-r)%360,c[1]=c[1]<0?c[1]+=360:c[1],n[1]=n[1]+r+v):o<=5?(c[0]=(c[0]+d)%360,n[0]=n[0]-d-v,c[1]=(c[1]+a)%360,n[1]=n[1]-a-v):o<=8?(d<=i?(c[0]=(c[0]+d)%360,n[0]=n[0]-d-v):(c[0]=(c[0]-i)%360,c[0]=c[0]<0?c[0]+=360:c[0],n[0]=n[0]+i+v),a<=r?(c[1]=(c[1]+a)%360,n[1]=n[1]-a-v):(c[1]=(c[1]-r)%360,c[1]=c[1]<0?c[1]+=360:c[1],n[1]=n[1]+r+v)):o<=11&&(i<=d?(c[0]=(c[0]+d)%360,n[0]=n[0]-d-v):(c[0]=(c[0]-i)%360,c[0]=c[0]<0?c[0]+=360:c[0],n[0]=n[0]+i+v),r<=a?(c[1]=(c[1]+a)%360,n[1]=n[1]-a-v):(c[1]=(c[1]-r)%360,c[1]=c[1]<0?c[1]+=360:c[1],n[1]=n[1]+r+v)),setHands(3*t+e,n[0],n[1],s)}}function setDigit(e,t,l,o){setHalfDigit(2*e,t.slice(0,3),l,o),setHalfDigit(2*e+1,t.slice(3,6),l,o)}function setStop(){for(let e=0;e<4;e++)setDigit(e,digit_stop,6,15)}function setLazy(t){for(let e=0;e<4;e++)setDigit(e,digits[t.charAt(e)],6,15)}function setFun(t){for(let e=0;e<4;e++)setDigit(e,digits[t.charAt(e)],1,16)}function setWaves(t){for(let e=0;e<4;e++)setDigit(e,digit_II,6,8);for(let e=0;e<8;e++)setTimeout(()=>{setHalfDigit(e,digit_II,1,18)},7e3+400*(e+1));for(let e=0;e<4;e++)setTimeout(()=>{setDigit(e,digits[t.charAt(e)],0,15)},16e3+400*(2*e+1))}let lastTime;function setTime(){var e=new Date,t=e.toTimeString().substring(0,5).replace(\":\",\"\"),l=(e.getDay()+6)%7,e=e.getHours();if(t!==lastTime&&0===sleep[l][e])switch(lastTime=t,selectedMode){case 0:setLazy(lastTime);break;case 1:setFun(lastTime);break;case 2:setWaves(lastTime)}else 1===sleep[l][e]&&setStop()}let clockInterval=void 0;function startClock(){void 0===clockInterval&&(lastTime=void 0,clockInterval=setInterval(setTime,500))}function stopClock(){clearInterval(clockInterval),clockInterval=void 0,setTimeout(setStop,100)}let lastClockInterval=void 0;document.addEventListener(\"visibilitychange\",e=>{\"visible\"==document.visibilityState?void 0!==lastClockInterval&&(clockInterval=setInterval(setTime,500)):(lastClockInterval=clockInterval,clearInterval(clockInterval))}),startClock()</script></body></html>"
#endif