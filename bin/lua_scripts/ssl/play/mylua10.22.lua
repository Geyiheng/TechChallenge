local slot=13/144--分子为两车宽，单位为厘米(若圆半径为144厘米)
local systemDelay=0.5--单位为秒
local cirR={}
      cirR[1]=172
      cirR[2]=150
      cirR[3]=172
local PgetBallflag=false
local cirR2=35

local interval=10
local move_threshold=3
 
local current_frame=0
local pos_table={}
local dist_table={}
local enemy_info_table={}
local Lpos=nil
local Apos=nil
local Spos=nil 
local idxpos=nil

local vacancyMinLength=6
local clearAreaR=60
local clearAreaR2=35
local maxShootDist=230
local chipCoef=1
local flag_firstShootDir=false




function degree_reverse(A)
  if A>0 then
    return -math.pi+A
  else
    return math.pi+A
  end
end

for i=0,param.maxPlayer do-----
  pos_table[i]={}
  dist_table[i]={}
  enemy_info_table[i]={}
  for j=0,interval do
    pos_table[i][j]=CGeoPoint:new_local(0,0)
    dist_table[i][j]=0
  end
  enemy_info_table[i]["dist"]=0
  enemy_info_table[i]["frame"]=0
end
 
function table_max_index(t)
  local max_value=0
  local max_index=0
  for k, v in pairs(t) do
    if max_value < v then
      max_value = v
      max_index=k
    end
  end
  return max_index
end
 
local function update_pos()
  for i=0,param.maxPlayer do
    pos_table[i][math.mod(current_frame,interval)]=enemy.pos(i)
  end
  current_frame=current_frame+1
end
 
local function update_dist()
  for i=0,param.maxPlayer do
    dist_table[i][math.mod(current_frame,interval)]=(pos_table[i][math.mod(current_frame,interval)]-pos_table[i][math.mod(current_frame+1,interval)]):mod()
  end
end
 
local function update_info()
  for i=0,param.maxPlayer do
    if current_frame-enemy_info_table[i]["frame"]<interval then
      if dist_table[i][math.mod(current_frame,interval)]>enemy_info_table[i]["dist"] then
        enemy_info_table[i]["dist"]=dist_table[i][math.mod(current_frame,interval)]
        enemy_info_table[i]["frame"]=current_frame
      end
    else
      enemy_info_table[i]["frame"]=table_max_index(dist_table[i])
      enemy_info_table[i]["dist"]=dist_table[i][enemy_info_table[i]["frame"]]
    end
  end
end
 -------------------------------------------------------------------------
local function nomove_enemy_list()
  local enemy_list={}
  for i=0,param.maxPlayer do
    if enemy.valid(i) then
      if enemy_info_table[i]["dist"]<move_threshold then
        table.insert(enemy_list,i)
      end
    end
  end
  return enemy_list
end
 ---------------------------------------------------------------------
function circleWall(player_d)
  return function()
      AoD=AoDdata(player_d)
      if AoD~=nil then
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-450+cirR[player_d]*math.cos(AoD),cirR[player_d]*math.sin(AoD)),"O")
        return CGeoPoint:new_local(-450+cirR[player_d]*math.cos(AoD),cirR[player_d]*math.sin(AoD))
      else
        debugEngine:gui_debug_msg(CGeoPoint:new_local(-450,0),"circleWallERROR_nilpoints")
      end
  end 
end
function AoDdata(player_d) --车号
  local AoD={}
  if WhoIsAboutToGetBall()~=nil then
    AoD[2]=(enemy.pos(WhoIsAboutToGetBall())-CGeoPoint:new_local(-450,0)):dir()
    debugEngine:gui_debug_msg(CGeoPoint:new_local(-100,300),"call_AboutToGetBall") 
  elseif EnemyBallPossession(false)<5 then
    AoD[2]=(enemy.pos(EnemyBallPossession(true))-CGeoPoint:new_local(-450,0)):dir()+prescient_Enemy()
     debugEngine:gui_debug_msg(CGeoPoint:new_local(-100,300),"call_followNextEnemy") 
  else 
    AoD[2]=(ball.pos()-CGeoPoint:new_local(-450,0)):dir()
    debugEngine:gui_debug_msg(ball.pos(),"call_SimplefollowBall") 
  end
  if ball.posX()<-360
    and ball.posY()>-90 
    and ball.posY()<90 then
    AoD[2]=0
  elseif AoD[2]>math.pi/2 then
    AoD[2]=math.pi/2
  elseif AoD[2]<-math.pi/2 then
    AoD[2]=-math.pi/2
  end
  if player_d==2 then
    return AoD[2]
  elseif player_d==1 then
    return AoD[2]-slot
  elseif player_d==3 then
    return AoD[2]+slot
  end
end 

-------------------------------------goalile

function WhoIsAboutToGetBall()
  none_move_enemy_list=nomove_enemy_list()
  for i=0,param.maxPlayer do
    if enemy.valid(i) then
      if enemy_info_table[i]["dist"]<move_threshold then
        table.insert(none_move_enemy_list,i)
      end
    end
  end
  local answer={}
  answer[1]=99999999
  answer[2]=99999999
  answer[3]=99999999
 for j=1,3 do
  if none_move_enemy_list[j]~=nil then
    local i=none_move_enemy_list[j]        
      if
      (enemy.pos(i)-ball.pos(i)):mod()<250
      and (enemy.pos(i)-ball.pos(i)):mod()>15
      and (ball.velDir()-0.17)<enemy.toBallDir_reverse(i) 
      and (enemy.toBallDir_reverse(i))<(ball.velDir()+0.17)
      and ball.velMod()>100
      then
      answer[j]=(math.abs(enemy.toBallDir_reverse(i)-ball.velDir())+0.1)*(math.abs(enemy.toBallDir_reverse(i)-enemy.dir(i))+0.1)/ball.velMod()*((enemy.pos(i)-ball.pos(i)):mod())^2---------------------------这里此段需要删除*(enemy.pos(i)-ball.pos(i)):mod()--
      end    
    end
  end
  if math.min(answer[1],answer[2],answer[3])~=99999999 then
    for j=1,3 do 
      if answer[j]==math.min(answer[1],answer[2],answer[3])  then
        debugEngine:gui_debug_msg(enemy.pos(none_move_enemy_list[j]),"enemy is about to get ball")
        return none_move_enemy_list[j]
      end
    end 
  end
end 



function ballIncoming()
  if degree_reverse(ball.velDir())>(ball.pos()-CGeoPoint:new_local(-450,50)):dir() 
    and degree_reverse(ball.velDir())<(ball.pos()-CGeoPoint:new_local(-450,-50)):dir()
    and ball.velMod()>100 then
    return 1
  else
    return nil
  end
end
function soonToShootBall()
none_move_enemy_list=nomove_enemy_list()
  local answer={}
  answer[1]=99999999
  answer[2]=99999999
  answer[3]=99999999
  for j=1,3 do
  if none_move_enemy_list[j]~=nil then
    local i=none_move_enemy_list[j]        
      if degree_reverse(enemy.dir(i))>(enemy.pos(i)-CGeoPoint:new_local(-450,50)):dir() 
     and degree_reverse(enemy.dir(i))<(enemy.pos(i)-CGeoPoint:new_local(-450,-50)):dir()
     and EnemyBallPossession(false)<5
     and enemy.rotVel(i)<10
      then
      answer[j]=(math.abs(degree_reverse(enemy.dir(i))-(enemy.pos(i)-CGeoPoint:new_local(-450,0)):dir())+0.01)*math.abs(degree_reverse(enemy.dir(i)))*(enemy.rotVel(i)+0.01)
      end    
    end
  end
  if math.min(answer[1],answer[2],answer[3])~=99999999 then
    for j=1,3 do 
      if answer[j]==math.min(answer[1],answer[2],answer[3])  then
        debugEngine:gui_debug_msg(enemy.pos(none_move_enemy_list[j]),"shoot ball imminent")
        return none_move_enemy_list[j]
      end
    end 
  else
    return nil
  end
end

function prescient_Enemy()
  if EnemyBallPossession(false)<8 then
  local whichEnemyPossessBall=EnemyBallPossession(true)
  local predictionDist=enemy.velMod(whichEnemyPossessBall)^2/8
  local v1=CVector:new_local(enemy.pos(whichEnemyPossessBall)-CGeoPoint:new_local(-450,0))
  local v2=CVector:new_local(predictionDist*math.cos(enemy.velDir(whichEnemyPossessBall)),predictionDist*math.sin(enemy.velDir(whichEnemyPossessBall)))
  local preEnemyDeltaDir=(v1+v2):dir()-v1:dir()
  debugEngine:gui_debug_msg(enemy.pos(EnemyBallPossession(true)),"prescient")
  return preEnemyDeltaDir*(1+enemy.velMod(whichEnemyPossessBall)/200)
  end
end

function PlayerBallPossession(getNumOrNot)---- true ----数字  false ---距离
  local PminDist=9999999999
  local Panswer={}
  for i=0,param.maxPlayer do
    if player.valid(gRoleName(i)) then
      if gRoleName(i)~=nil then
      Panswer[i]=ball.toPlayerHeadDist(gRoleName(i))
        if PminDist>Panswer[i] then
          PminDist=Panswer[i]
        end
      end
    end
  end
  if PminDist<5 and PgetBallflag==false then
    Lpos=player.pos("Leader")
    Apos=player.pos("Assister")
    Spos=player.pos("Special")
    PgetBallflag=true
  end
  if PminDist>8 then
    PgetBallflag=false 
  end
  if getNumOrNot==true then
    return idx
  else
    return PminDist
  end
end

function EnemyBallPossession(getNumOrNot)---- true ----数字  false ---距离
  local EminDist=9999999999
  local Eanswer={}
  for i=0,param.maxPlayer do
    if enemy.valid(i) then
      Eanswer[i]=ball.toEnemyHeadDist(i)
      if EminDist>Eanswer[i] then
        EminDist,idx=Eanswer[i],i
      end
    end
  end
  if getNumOrNot==true then
    return idx
  else
    return EminDist
  end
end

function turnAroundDir(role) -----角色----
  return function()
    local minDist=9999999999
    local answerDist={}
    for i=0,param.maxPlayer do
      if enemy.valid(i) then
        answerDist[i]=(enemy.pos(i)-player.pos(role)):mod()
        if minDist>answerDist[i] then
          minDist,idxDist=answerDist[i],i
        end
      end
    end
    return (player.pos(role)-enemy.pos(idxDist)):mod()
  end
end

function DoDdata(role) -------带球防御
  local DoD={}
  DoD[2]=(enemy.pos(EnemyBallPossession(true))-ball.pos()):dir()--+prescient_Enemy_forAdvavnce()
    
  if DoD[2]==nil then
    DoD[2]=0
  end
  
  if role=="Leader" then
    return DoD[2]-math.pi/12
  elseif role=="Assister" then
    return DoD[2]
  elseif role=="Special" then
    return DoD[2]+math.pi/12
  end
end
function advancePos(role)
  return function()
    DoD=DoDdata(role)
    return CGeoPoint:new_local(ball.posX()+cirR2*math.cos(DoD),ball.posY()+cirR2*math.sin(DoD))
  end
end


function DoKdata(myRole,idx)
  local DoK={}
   DoK[0]=(CGeoPoint:new_local(450,-50)-player.pos(myRole)):dir()
   DoK[10]=(CGeoPoint:new_local(450,50)-player.pos(myRole)):dir()
  DoK[idx]=(DoK[10]-DoK[0])*idx/10+DoK[0]
  if DoK[idx]~=nil then
     -- debugEngine:gui_debug_msg(CGeoPoint:new_local(450,0),tostring(DoK[1])..","..tostring(DoK[10]))
    return DoK[idx] 
  end
end

function MyChaseDir(DoKorCnt,myRole,playerRole1,playerRole2) --（true--角度、false--Spectrum 的数量，拿球人名字。
  return function()


    local vacancy={}
    for j=1,10 do 
      vacancy[j]=true--true为空着
    end
    for i=0,15 do 
      if enemy.valid(i) then 
        for j=1,10 do
          if DoKdata(myRole,j-1)~=nil and DoKdata(myRole,j)~=nil 
          and DoKdata(myRole,j-1)<=(enemy.pos(i)-player.pos(myRole)):dir()
          and (enemy.pos(i)-player.pos(myRole)):dir()<=DoKdata(myRole,j) then
          vacancy[j]=false
          else
          vacancy[j]=true--true为空着
          end
        end
      end
    end

    if playerRole1~=nil then
      for j=1,10 do
        if DoKdata(myRole,j-1)~=nil and DoKdata(myRole,j)~=nil 
        and DoKdata(myRole,j-1)<=(player.pos(playerRole1)-player.pos(myRole)):dir()
        and (player.pos(playerRole1)-player.pos(myRole)):dir()<=DoKdata(myRole,j) then
        vacancy[j]=false
        else
        vacancy[j]=true--true为空着
        end
      end
    end

    if playerRole2~=nil then
      for j=1,10 do
        if DoKdata(myRole,j-1)~=nil and DoKdata(myRole,j)~=nil 
        and DoKdata(myRole,j-1)<=(player.pos(playerRole2)-player.pos(myRole)):dir()
        and (player.pos(playerRole2)-player.pos(myRole)):dir()<=DoKdata(myRole,j) then
        vacancy[j]=false
        else
        vacancy[j]=true--true为空着
        end
      end
    end

    vacancy[11]=false
    local count=0
    local maxcount=0
    for j=1,10 do 
      if vacancy[j]==false then
        count=0
      elseif vacancy[j]==true then
        count=count+1
      end
      if count>maxcount then
        maxcount=count
      end
    end
    local DoKans=0
    if maxcount>0 then
      for search=1,11-maxcount do
        flag_continue=true
        for searcheach=search,search+maxcount-1 do
          if vacancy[searcheach]==false then
            flag_continue=false
          end
        end
        if flag_continue==true then
          DoKans=(DoKdata(myRole,search-0.5)+DoKdata(myRole,search+maxcount-1.5))/2
          break 
        end
      end
    end
    debugEngine:gui_debug_msg(CGeoPoint:new_local(450,0),tostring(flag_continue))
    if DoKorCnt==true then
      return DoKans
    else  
      return maxcount
    end
  end
end
function gRoleName(i)
  if gRoleNum["Leader"]==i then
    return "Leader"
  elseif gRoleNum["Assister"]==i then
    return "Assister"
  elseif gRoleNum["Special"]==i then
    return "Special"
  else
    return "Goalie"
  end
end

function getPos(func)
  return function()
    print("getpos is running")
    local i=func()
    if i==-1 or i==nil then
      return CGeoPoint:new_local(0,0)
    else
   
      return player.pos(func())
    end
  end
end

function chipPassRole(InOrOut) ----true---in  false---out 
  return function ()
    -- if PlayerBallPossession(false)<10 and EnemyBallPossession(false)>20 then --放到状态机
    if InOrOut==true then 
      clearAreaR_local=clearAreaR
    else
      clearAreaR_local=clearAreaR2
    end
    local clearAround={}
    for i=0,15 do 
      if player.valid(gRoleName(i)) then
        if PlayerBallPossession(true)~=i then
        clearAround[i]=true
          for j=0,15 do
            if enemy.valid(j) then
              if (enemy.pos(j)-player.pos(i)):mod()<=clearAreaR_local then
            clearAround[i]=false
              end
            end
          end
          for o=1,2 do
            if playerWithoutBall(o)~=nil then
              if (player.pos(PlayerBallPossession(true))-player.pos(playerWithoutBall(o))):mod()<clearAreaR_local then
                clearAround[i]=false 
              end
            end
          end
        end
      end
    end

    
    local minToGoalDist=9999999999
    local minidx=-1
    for i=0,15 do
      if player.valid(gRoleName(i)) then
        if PlayerBallPossession(true)~=i then
          if clearAround[i]==true then
           local FreePtoGoalDist=(CGeoPoint:new_local(450,0)-player.pos(gRoleName(i))):mod()--player.pos("Leader")            
            if minToGoalDist>FreePtoGoalDist then
          minToGoalDist,minidx=(CGeoPoint:new_local(450,0)-player.pos(gRoleName(i))):mod(),i--player.pos(gRoleName(i))-CGeoPoint:new_local(450,0)
            end
          end
        end
      end
    end
    debugEngine:gui_debug_msg(CGeoPoint:new_local(0,0),tostring(gRoleName(chipPassRole(false))))
    return minidx
  end
end


function playerWithoutBall(idx)
  local playerWithoutBallList={}
  for i=0,15 do

    if player.valid(gRoleName(i)) then
      if i~=PlayerBallPossession(true) then
      table.insert(playerWithoutBallList,i)
      end
    end
  end
  return playerWithoutBallList[idx]
end
function Ymove(myRole_nad,X)
  return function()
    if X<0 then Y=290
    elseif X<100 then Y=250
    elseif X<200 then Y=180
    elseif X<300 then Y=150
    else Y=100
    end
    if player.posY(myRole_nad)>Y-10 then
      return CGeoPoint:new_local(X,-Y)
    elseif player.posY(myRole_nad)<-Y+10 then
      return CGeoPoint:new_local(X,Y)
    elseif flag_firstSwing==true then
      flag_firstSwing=false
      return CGeoPoint:new_local(X,-Y)
    else
      return CGeoPoint:new_local(260,0)
    end
  end
end
-- function chipPower(myRole)
--   return function()
--     return (player.pos(chipPassRole())-player.pos(myRole)):mod()*chipCoef
--   end
-- end
      
local ShootPos1  = CGeoPoint:new_local(250,150)
local ShootPos2  = CGeoPoint:new_local(250,-150)


gPlayTable.CreatePlay{
 
firstState = "goalie",

["goalie"] = {
  switch = function ()
    flag_InAdvance=0
    flag_firstSwing=false
    update_pos()
    update_dist()
    update_info()

    if bufcnt(PlayerBallPossession(false)<30,5) or  bufcnt(soonToShootBall()~=nil,3) then--getPlayerBallPossession_no_rolenum() or
      return "getBall"
    end
  end,
Leader=task.goCmuRush(circleWall(1),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
Assister=task.goCmuRush(circleWall(2),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
Special=task.goCmuRush(circleWall(3),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
match="[LSA]"
},

  
["getBall"] = {

  switch = function ()
    flag_InAdvance=0
    flag_firstSwing=false
    update_pos()
    update_dist()
    update_info()
    if bufcnt(EnemyBallPossession(false)>15 and PlayerBallPossession(false)<5,2) then
      flag_firstSwing=true
      if (ball.posX()>200 and ball.posY()>-200 and ball.posY()<200)then
        return "shoot"
      else
        return "pass"
      end
    -- elseif Lpos~=nil then
    --   if ((Lpos-player.pos("Leader")):mod()>90 or (Apos-player.pos("Assister")):mod()>90 or (Spos-player.pos("Special")):mod()>90) then
    --flag_firstSwing=true
    --     return "moveOrShoot"
    -- end
    end
  end,

Leader=task.getBall(ball.pos()),
Assister=task.getBall(ball.pos()),
Special=task.getBall(ball.pos()),
match="[LSA]"
},

-- ["moveOrShoot"] = {
--   switch = function ()
--     if PlayerBallPossession(false)>10 and EnemyBallPossession(false)<15 then
--       return "getBall"
--     elseif MyChaseDir(false,PlayerBallPossession(true),playerWithoutBall(1),playerWithoutBall(2))()>vacancyMinLength 
--       and (player.pos(PlayerBallPossession(true))-CGeoPoint:new_local(450,0)):mod()<maxShootDist  then
--       flag_firstShootDir=true
--       shootDir_G=MyChaseDir(true,PlayerBallPossession(true),playerWithoutBall(1),playerWithoutBall(2))
--       return "shoot"
--     elseif bufcnt(chipPassRole(true)()~=-1,2) then
--       return "pass" 
--     end
--   end,
--   Leader=task.getBall(ball.pos()),--goCmuRush(Ymove("Leader",0))
--   Assister=task.goCmuRush(Ymove("Assister",150)),
--   Special=task.goCmuRush(Ymove("Special",280)),
--   match="[LSA]"

-- },


["pass"]={
  switch= function ()
    flag_firstSwing=false
    if PlayerBallPossession(false)>10 and EnemyBallPossession(false)<15 then
      return "getBall"
    elseif (ball.posX()>200 and ball.posY()>-200 and ball.posY()<200) and 
      MyChaseDir(false,PlayerBallPossession(true),playerWithoutBall(1),playerWithoutBall(2))()>vacancyMinLength then
      return "shoot"
    -- elseif bufcnt(chipPassRole(false)()==-1,1,250) then--
    --   flag_firstSwing=true
    --   return "moveOrShoot"
    end
  end,--chipPassRole()

  Leader=task.chipPass(getPos(chipPassRole(false)),300),
  Assister=task.goCmuRush(ShootPos1),
  Special=task.goCmuRush(ShootPos2),
  match="{L}[SA]"
-- 
},

-- ["receive"]={

-- }

["shoot"] = {

  switch = function ()
    flag_InAdvance=0
    flag_firstSwing=false
    update_pos()
    update_dist()
    update_info()
    -- MyChaseDir(true,"Assister","Leader","Special")()
    if bufcnt(EnemyBallPossession(false)<6,2) or (ball.posX()<200 or ball.posY()<-200 or ball.posY()>200 or ball.posX()>350)then
      return "getBall"
    elseif MyChaseDir(false,PlayerBallPossession(true),playerWithoutBall(1),playerWithoutBall(2))()<=vacancyMinLength-2 then
      flag_firstSwing=true
      return "pass"
    end
  end,
Leader=task.chaseNew(),--goCmuRush(advancePos("Leader"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
-- Assister=task.passToDir(0,550),--passToDir(DoKdata("Assister",4)),--goCmuRush(advancePos("Assister"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
Assister=task.chaseNew(),--passToDir(DoKdata("Assister",4)),--goCmuRush(advancePos("Assister"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
Special=task.chaseNew(),--goCmuRush(advancePos("Special"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
match="[LAS]"
},
-- passToDir(MyChaseDir(true,"Assister","Leader","Special"),550),--

name = "mylua10.22",
applicable ={
  exp = "a",
  a = true
},
attribute = "attack",
timeout = 99999
}
 
  -- ["turnAround"] = {
--   switch = function ()
--     update_pos()
--     update_dist()
--     update_info()
--     if bufcnt(EnemyBallPossession(false)<8,2) then
--       return "getBall";
--     end
--   end,
-- Leader=task.goCmuRush(player.pos("Leader"),turnAroundDir("Leader")),--passToPos(CGeoPoint:new_local(300,0),5),
-- Assister=task.goCmuRush(player.pos("Assister"),turnAroundDir("Assister")),--passToPos(CGeoPoint:new_local(300,0),5),
-- Special=task.goCmuRush(player.pos("Special"),turnAroundDir("Special")),--passToPos(CGeoPoint:new_local(300,0),5),
-- match="[LSA]"
-- },
-- ["advance"]={
--   switch = function ()
--     update_pos()
--     update_dist()
--     update_info()
--     flag_InAdvance=flag_InAdvance+1
--     -- if bufcnt(false,1,180) then
--     --   return "pass1"
--     if flag_InAdvance==1 then
--     Lpos=player.pos("Leader")
--     Apos=player.pos("Assister")
--     Spos=player.pos("Special")
--     end
--     if bufcnt(EnemyBallPossession(false)<6,2) or ((Lpos-player.pos("Leader")):mod()>90 or (Apos-player.pos("Assister")):mod()>90 or (Spos-player.pos("Special")):mod()>90) then
--       return "getBall"
--     elseif player.posX("Assister")>290  then
--       return "shoot"
--     end
--   end,
-- Leader=task.goCmuRush(advancePos("Leader"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
-- Assister=task.passToPos(CGeoPoint:new_local(335,0),20),--goCmuRush(advancePos("Assister"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
-- Special=task.goCmuRush(advancePos("Special"),_,_,bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)),
-- match="[A][LS]"
-- },







--if (player.pos("Leader")-ball.pos()):mod()<20 or(player.pos("Assister")-ball.pos()):mod()<20 or (player.pos("Special")-ball.pos()):mod()<20  then
   --    return "goalie_dribbling";
   --  end
-- ["goalie_dribbling"]={
-- switch = function ()
--   update_pos()
--   update_dist()
--   update_info()
--   if (player.pos("Leader")-ball.pos()):mod()>=20 or(player.pos("Assister")-ball.pos()):mod()>=20 or (player.pos("Special")-ball.pos()):mod()>=20  then
--     return "goalie";
--   elseif bufcnt(BallPossession(true,gRoleNum("Leader"))>0.3,  
--     end
--   end,
-- Leader=task.goCmuRush(circleWall(1),_,_,bit:_or(flag.not_avoid_our_vehicle,bit:_or(flag.not_avoid_their_vehicle,flag.dribbling))),
-- Assister=task.goCmuRush(circleWall(2),_,_,bit:_or(flag.not_avoid_our_vehicle,bit:_or(flag.not_avoid_their_vehicle,flag.dribbling))),
-- Special=task.goCmuRush(circleWall(3),_,_,bit:_or(flag.not_avoid_our_vehicle,bit:_or(flag.not_avoid_their_vehicle,flag.dribbling))),
-- match="[LSA]"static
-- },