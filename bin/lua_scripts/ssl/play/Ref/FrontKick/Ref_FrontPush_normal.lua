local WAIT_BALL_POS   = function ()
  return ball.pos() + Utils.Polar2Vector(50, ball.syntY(0.3 * math.pi))
end

--【接球点】可根据实车情况进行调整
local RECEIVE_POS = ball.antiYPos(CGeoPoint:new_local(0, 100))
--【传球力度】可根据实车情况进行调整
local kickPower = 400

gPlayTable.CreatePlay{

  firstState = "start",

  ["start"] = {
    switch = function ()
      if bufcnt(player.toTargetDist("Assister") and player.toTargetDist("Middle") < 20 , 10, 180) then
        return "toBall"
      end
    end,
    Assister = task.goCmuRush(WAIT_BALL_POS,_,_,flag.allow_dss + flag.dodge_ball),
    Leader   = task.markingFront("First"),
    Middle   = task.markingFront("Second"),
    Special  = task.multiBack(3,1),
    Defender = task.multiBack(3,2),
    Breaker  = task.multiBack(3,3),
    Crosser  = task.defendHead(),
    Goalie   = task.goalieNew(),
    match = "{S}{A}{D}{C}{B}{LM}"
  },

  ["toBall"] = {--不拿球
    switch = function ()
      if bufcnt(player.toPointDist("Assister", ball.pos()) < 10, 3, 180) then
        return "kickBall"
      end
    end,
    Assister = task.staticGetBall(ball.pos()),
    Leader   = task.markingFront("First"),
    Middle   = task.markingFront("Second"),
    Special  = task.goCmuRush(RECEIVE_POS,player.toBallDir("Special"),_,flag.allow_dss),
    Defender = task.multiBack(2,1),
    Breaker  = task.multiBack(2,2),
    Crosser  = task.defendHead(),
    Goalie   = task.goalieNew(),
    match = "{D}{A}{B}{S}{C}{LM}"
  },

  ["kickBall"] = {
    switch = function ()
      if bufcnt(player.kickBall("Assister"), "fast", 180) then
        return "receiveBall"
      end
    end,
    Assister = task.passToPos(RECEIVE_POS, kickPower),
    Leader   = task.markingFront("First"),
    Middle   = task.markingFront("Second"),
    Special  = task.goCmuRush(RECEIVE_POS,player.toBallDir("Special"),_,flag.allow_dss),
    Defender = task.multiBack(2,1),
    Breaker  = task.multiBack(2,2),
    Crosser  = task.defendHead(),
    Goalie   = task.goalieNew(),
    match = "{D}{A}{B}{S}{C}{LM}"
  },

  ["receiveBall"] = {
    switch = function ()
      if bufcnt(player.toBallDist("Special")<15, 3, 180) then--
        return "shootBall"
      end
    end,
    Assister = task.markingFront("Third"),
    Leader   = task.markingFront("First"),
    Middle   = task.markingFront("Second"),
    Special  = task.receive(ball.pos()),
    Defender = task.multiBack(2,1),
    Breaker  = task.multiBack(2,2),
    Crosser  = task.defendHead(),
    Goalie   = task.goalieNew(),
    match = "{D}{A}{B}{S}{C}{LM}"
  },

["shootBall"] = {
    switch = function ()
      if bufcnt(player.kickBall("Special"), 3, 180) then--
        return "exit"
      end
    end,
    Assister = task.markingFront("Third"),
    Leader   = task.markingFront("First"),
    Middle   = task.markingFront("Second"),
    Special  = task.chaseNew(),
    Defender = task.multiBack(2,1),
    Breaker  = task.multiBack(2,2),
    Crosser  = task.defendHead(),
    Goalie   = task.goalieNew(),
    match = "{D}{A}{B}{S}{C}{LM}"
  },


  name = "Ref_FrontPush_normal",
  applicable = {
    exp = "a",
    a   = true
  },
  attribute = "attack",
  timeout = 99999

}
