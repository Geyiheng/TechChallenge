
gPlayTable.CreatePlay{

  firstState = "start",

  ["start"]= {
  switch = function()
      if cond.isGameOn() then
        return "exit"
      end
    end,
  Leader = task.defendKick(),
  Assister = task.defendHead(),
  Special = task.marking("First"),
  Middle = task.leftBack(),
  Defender = task.rightBack(),
  Goalie = task.goalieNew(),
  match    = "[L][A][MD][S]"
},
  name = "Ref_CornerDefV2",
  applicable = {
    exp = "a",
    a = true
  },
  attribute = "defense",
  timeout = 99999
}