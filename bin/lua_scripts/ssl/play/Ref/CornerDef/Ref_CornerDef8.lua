gPlayTable.CreatePlay{

  firstState = "start",

  ["start"]= {
  switch = function()
      if cond.isGameOn() then
        return "exit"
      end
    end,
  Leader = task.multiBack(1,3),
  Assister = task.defendKick(),
  Special = task.markingFront("First"),
  Breaker = task.markingFront("Second"),
  Crosser = task.markingFront("Third"),
  Middle = task.multiBack(2,3),
  Defender = task.multiBack(3,3),
  Goalie = task.goalieNew(),
  match    = "[A]" -- for test
},
  name = "Ref_CornerDef8",
  applicable = {
    exp = "a",
    a = true
  },
  attribute = "defense",
  timeout = 99999
}