
local function def_chipPower()
  if math.abs(ball.posY()) > 270 then 
    return 180
  else 
    return 100
  end
end

local ANTI_POS_1 = ball.refAntiYPos(CGeoPoint:new_local(112, 33))
local ANTI_POS_2 = ball.refAntiYPos(CGeoPoint:new_local(225, 66))
local ANTI_POS_3 = ball.refAntiYPos(CGeoPoint:new_local(337, 86))


local SYNT_POS_1 = ball.refSyntYPos(CGeoPoint:new_local(150, 33))
local SYNT_POS_2 = ball.refSyntYPos(CGeoPoint:new_local(300, 40))
local SYNT_POS_3 = ball.refSyntYPos(CGeoPoint:new_local(225,120))


local SHOOT_POS = pos.passForTouch(ball.refAntiYPos(CGeoPoint:new_local(300, 86)))

local dangerous = true


gPlayTable.CreatePlay{

  firstState = "start",

  ["start"] = {
    switch = function ()
      if bufcnt(player.toTargetDist("Defender") < 20 , 10, 180) then
        return "passBall"
      end
    end,
    Assister = task.staticGetBall(SHOOT_POS()),
    Leader   = task.rightBack(),
    Special  = task.leftBack(),
    Middle   = task.goCmuRush(SYNT_POS_1,_,_,flag.allow_dss),
    Defender = task.goCmuRush(ANTI_POS_1,_,_,flag.allow_dss),
    Goalie   = dangerous and task.goalieNew(),
    Engine   = task.goLWPassPos("Assister"),
    Hawk     = task.goCmuRush(ANTI_POS_3,_,_,flag.allow_dss),
    match    = "{A}{LS}{M}{DEH}"
  },


  ["passBall"] = {
    switch = function ()
      --chip_power_func()
      if player.kickBall("Assister") or player.toBallDist("Assister") > 30 then
        return "waitBall"
      end
    end,
    Assister = task.chipPass(SHOOT_POS(),def_chipPower()),
    Leader   = task.rightBack(),
    Special  = task.leftBack(),
    Middle   = task.goCmuRush(ANTI_POS_2,_,_,flag.allow_dss),
    Defender = task.goCmuRush(SYNT_POS_2,_,_,flag.allow_dss),
    Goalie   = task.goalieNew(),
    Engine   = task.goLWPassPos("Assister"),
    Hawk     = task.goCmuRush(SYNT_POS_3,_,_,flag.allow_dss),
    match    = "{A}{LS}{M}{DEH}"
  },


  ["waitBall"] = {
    switch = function ()
      print("waitBall")
      if bufcnt(player.toPointDist("Defender", SYNT_POS_2) < 30, 3, 60) then
        return "shoot"
      end
    end,
    Assister = task.stop(),
    Leader   = task.rightBack(),
    Special  = task.leftBack(),
    Middle   = task.goCmuRush(ANTI_POS_3,_,_,flag.allow_dss),
    Defender = task.continue(),
    Goalie   = task.goalieNew(),
    Engine   = task.goLWPassPos("Assister"),
    Hawk     = task.continue(),
    match    = "{A}{LS}{M}{DEH}"
  },

  ["shoot"] = {
    switch = function ()
      print("shoot")
      if bufcnt(player.kickBall("Defender"), "fast", 180) then
        print("exit")
        return "exit"
      end
    end,
    Assister = task.stop(),
    Leader   = task.rightBack(),
    Special  = task.leftBack(),
    Middle   = task.shootV2(),
    Defender = task.goLeftSupport(),
    Goalie   = task.goalieNew(),
    Engine   = task.goLWPassPos("Assister"),
    Hawk     = task.goRightSupport(),
    match    = "{A}{LS}{M}{DEH}"
  },

  name = "Ref_CornerKickV8",
  applicable = {
    exp = "a",
    a   = true
  },
  attribute = "attack",
  timeout = 99999

}
