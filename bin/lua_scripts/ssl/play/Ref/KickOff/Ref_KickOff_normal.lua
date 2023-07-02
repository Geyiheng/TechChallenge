--[[-- 直接传球
-- 在改一下V1

local CHIP_POS = CGeoPoint:new_local(200/1200*param.pitchLength,200/900*param.pitchWidth)

local ASSISTER_POS_1 = CGeoPoint:new_local(-15/1200*param.pitchLength,230/900*param.pitchWidth)
local ASSISTER_POS_2 = CGeoPoint:new_local(-60/1200*param.pitchLength,60/900*param.pitchWidth)

local SPECIAL_POS_1  = CGeoPoint:new_local(-15/1200*param.pitchLength,-230/900*param.pitchWidth)
local SPECIAL_POS_2  = CGeoPoint:new_local(200/1200*param.pitchLength,-60/900*param.pitchWidth)

local MIDDLE_POS_1   = CGeoPoint:new_local(-220/1200*param.pitchLength,250/900*param.pitchWidth)
local MIDDLE_POS_2   = CGeoPoint:new_local(-5/1200*param.pitchLength,250/900*param.pitchWidth)


gPlayTable.CreatePlay{
firstState = "start",

["start"] = {
  switch = function ()
    if cond.isNormalStart() then
      return "temp"
    end
  end,
  Leader   = task.staticGetBall(CHIP_POS),
  Assister = task.goCmuRush(ASSISTER_POS_1),
  Special  = task.goCmuRush(SPECIAL_POS_1),
  Middle   = task.goCmuRush(MIDDLE_POS_1),
  Defender = task.singleBack(),
  Goalie   = task.goalieNew(),
  match    = "{LASMD}"
},

["temp"] = {
  switch = function ()
    if bufcnt(player.toPointDist("Assister", ASSISTER_POS_1) < 10 and player.toPointDist("Special", SPECIAL_POS_1) < 10, 60,100) then
      return "chip"
    end
  end,
  Leader   = task.staticGetBall(CHIP_POS),
  Assister = task.goCmuRush(ASSISTER_POS_1),
  Special  = task.goCmuRush(SPECIAL_POS_1),
  Middle   = task.goSpeciPos(MIDDLE_POS_1),
  Defender = task.singleBack(),
  Goalie   = task.goalieNew(),
  match    = "{LASMD}"
},


["chip"] = {
    switch = function ()
    --if bufcnt(player.kickBall("Leader"), "fast", 200) then
      if bufcnt(player.kickBall("Leader"), "fast", 200) then
      return "receive"
    end
  end,
  Leader   = task.goAndTurnKick("Assister", 500),
  Special  = task.goCmuRush(SPECIAL_POS_1),
  Middle   = task.defendMiddle(),
  Defender = task.rightBack(),
  Assister = task.goCmuRush(ASSISTER_POS_1),
  Goalie   = task.goalieNew(),
  match    = "{LASMD}"
},

["receive"] = {
  switch = function ()
    if bufcnt(player.toBallDist("Assister") < 20, "fast", 150) then
      return "exit"
    end
  end,
  Leader   = task.goCmuRush(CGeoPoint:new_local(200/1200*param.pitchLength,20/900*param.pitchWidth)),
  Special  = task.goCmuRush(SPECIAL_POS_2),
  Middle   = task.defendMiddle(),
  Defender = task.rightBack(),
  Assister = task.receiveShoot(),
  Goalie   = task.goalieNew(),
  match    = "{LASMD}"
},



name = "Ref_KickOffV6",
applicable = {
  exp = "a",
  a = true
},
attribute = "attack",
timeout = 99999
}
--]]
local LEFT_POS = CGeoPoint:new_local(200/1200*param.pitchLength,-165/900*param.pitchWidth)
local LEFT_POS_2 = CGeoPoint:new_local(300/1200*param.pitchLength,-165/900*param.pitchWidth)

local START_POS = CGeoPoint:new_local(27/1200*param.pitchLength,18/900*param.pitchWidth)
local START_POS_1 = CGeoPoint:new_local(9/1200*param.pitchLength,6/900*param.pitchWidth)
local RECEIVE_POS_1 = CGeoPoint:new_local(-110/1200*param.pitchLength,-70/900*param.pitchWidth)
local RECEIVE_POS_2 = CGeoPoint:new_local(-110/1200*param.pitchLength,70/900*param.pitchWidth)

local RIGHT_POS_1  = CGeoPoint:new_local(-20/1200*param.pitchLength,80/900*param.pitchWidth)
local RIGHT_POS_2  = CGeoPoint:new_local(155/1200*param.pitchLength,80/900*param.pitchWidth)
local outside = CGeoPoint:new_local(-50/1200*param.pitchLength,300/900*param.pitchWidth)

local GetBallPos = pos.passForTouch(RECEIVE_POS_1)
local GetBallPos1 = CGeoPoint:new_local(-110/1200*param.pitchLength,70/900*param.pitchWidth)

local getKickerNum = function()
    return "Leader"
end

local receive_dir = function(num)
  return (player.pos(gRoleNum[getKickerNum()]) - player.pos(num)):dir()
end

gPlayTable.CreatePlay{
firstState = "start",

["start"] = {
  switch = function ()
    if cond.isNormalStart() then
      return "getball"
    end
  end,
  Leader   = task.goCmuRush(START_POS,math.pi+0.5,_,flag.allow_dss),
  Assister = task.goCmuRush(RECEIVE_POS_1,receive_dir,_,flag.allow_dss),
  Special  = task.goCmuRush(RECEIVE_POS_2,receive_dir,_,flag.allow_dss),
  Middle   = task.multiBack(4,1),
  Defender = task.multiBack(4,2),
  Breaker  = task.multiBack(4,3),
  Crosser  = task.multiBack(4,4),
  Goalie   = task.goalieNew(),
  match    = "[L][A][S][MDBC]"
},

["temp"] = {
  switch = function ()
    if bufcnt(player.toTargetDist("Leader")<20 ,50,100) then
      return "getball"
    end
  end,
  Leader   = task.goCmuRush(START_POS_1,math.pi+0.5,_,flag.allow_dss),  --staticGetBall(player.toPlayerDir("Assister","Leader")),
  Assister = task.goCmuRush(RECEIVE_POS_1,receive_dir,_,flag.allow_dss),
  Special  = task.goCmuRush(RECEIVE_POS_2,receive_dir,_,flag.allow_dss),
  Middle   = task.multiBack(4,1),
  Defender = task.multiBack(4,2),
  Breaker  = task.multiBack(4,3),
  Crosser  = task.multiBack(4,4),
  Goalie   = task.goalieNew(),
  match    = "[L][A][S][MDBC]"
},

["getball"] = {
  switch = function ()
    if bufcnt(player.toTargetDist("Leader")<20 ,50,100)  then
      if ball.posY() > 0 then  --and ball.posY() < 0
        return "kickoftoA"
    else
        return "kickoftoS"
      end
    end
  end,
  Leader   = task.staticGetBall(GetBallPos1), --task.staticGetBall(RECEIVE_POS_1),
  Assister = task.goCmuRush(RECEIVE_POS_1,receive_dir,_,flag.allow_dss),
  Special  = task.goCmuRush(RECEIVE_POS_2,receive_dir,_,flag.allow_dss),
  Middle   = task.multiBack(4,1),
  Defender = task.multiBack(4,2),
  Breaker  = task.multiBack(4,3),
  Crosser  = task.multiBack(4,4),
  Goalie   = task.goalieNew(),
  match    = "[L][A][S][MDBC]"
},



["kickoftoA"] = {
    switch = function ()
    --if bufcnt(player.kickBall("Leader"), "fast", 200) then
      if bufcnt(player.toBallDist("Leader") > 30) then
      return "receive"
    end
  end,
  Leader   = task.passToPos(RECEIVE_POS_1,350),--
  Assister = task.goCmuRush(RECEIVE_POS_1,receive_dir,_,flag.allow_dss),
  Special  = task.goCmuRush(RECEIVE_POS_2,receive_dir,_,flag.allow_dss),
  Middle   = task.multiBack(4,1),
  Defender = task.multiBack(4,2),
  Breaker  = task.multiBack(4,3),
  Crosser  = task.multiBack(4,4),
  Goalie   = task.goalieNew(),
  match    = "[L][A][S][MDBC]"
},

["kickoftoS"] = {
    switch = function ()
    --if bufcnt(player.kickBall("Leader"), "fast", 200) then
      if bufcnt(player.toBallDist("Leader") > 30) then
      return "receive"
    end
  end,
  Leader   = task.passToPos(RECEIVE_POS_2,350),--
  Assister = task.goCmuRush(RECEIVE_POS_1,receive_dir,_,flag.allow_dss),
  Special  = task.goCmuRush(RECEIVE_POS_2,receive_dir,_,flag.allow_dss),
  Middle   = task.multiBack(4,1),
  Defender = task.multiBack(4,2),
  Breaker  = task.multiBack(4,3),
  Crosser  = task.multiBack(4,4),
  Goalie   = task.goalieNew(),
  match    = "[L][A][S][MDBC]"
},

["receive"] = {
  switch = function ()
    if bufcnt(player.toBallDist("Assister") < 20, 20, 150) then
      return "exit"
    end
  end,
  Leader   = task.multiBack(4,1),
  Assister = task.receive("Leader"),
  Special  = task.leftBack(),
  Middle   = task.rightBack(),
  Defender = task.multiBack(4,2),
  Breaker  = task.multiBack(4,3),
  Crosser  = task.multiBack(4,4),
  Goalie   = task.goalieNew(),
  match    = "[A][SM][LDBC]"
},


name = "Ref_KickOff_normal",
applicable = {
  exp = "a",
  a = true
},
attribute = "attack",
timeout = 99999
}
