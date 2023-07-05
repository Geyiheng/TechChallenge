local DetectTheSupportID = function() 
    if ball.posX() > 200 and ball.posY() < -120 then  -- 在0号区域
        return 2  -- 往2号区域打
    elseif ball.posX() > 200 and ball.posY() < 120 then -- 在1号区域
        if ball.posX() > 0 then
            return 2
        else 
            return 0
        end
    elseif ball.posX() > 200 and ball.posY() >= 120 then -- 在2号区域
        return 0 -- 往0号打
    elseif ball.posX() > -200 and ball.posY() < -120 then -- 在3号区域
        return 0 -- 往0号打
    elseif ball.posX() > -200 and ball.posY() < 120 then -- 在4号区域
        return 1 -- 往1号打
    elseif ball.posX() > -200 and ball.posY() >= 120 then -- 在5号区域
        return 2 -- 往2号打
    else
        return 4 -- 往4号打
    end
end

gPlayTable.CreatePlay{

  firstState = "start",

  ["start"]= {
  switch = function()
      if cond.isGameOn() then
        return "exit"
      end
    end,
  Leader   = task.defendKick(),
  Assister = task.markingFront("First"),
  Middle   = task.sideBack(),
  Special  = task.support("Leader",DetectTheSupportID), 
  Defender = task.multiBack(3,1),
  Crosser  = task.multiBack(3,2),
  Breaker  = task.multiBack(3,3),--sideBack_Ultra
  Goalie   = task.goalieNew(),
  match    = "[L][DC][S][A][B][M]"
},
  name = "Ref_BackDef_normal",
  applicable = {
    exp = "a",
    a = true
  },
  attribute = "defense",
  timeout = 99999
}