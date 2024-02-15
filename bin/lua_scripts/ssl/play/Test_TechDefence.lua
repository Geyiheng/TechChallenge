gPlayTable.CreatePlay{

	firstState = "start",

	["start"] = {
    switch = function ()
      if true then
        return "start"
      end
    end,
    Special  = task.techdefence(1),
    Assister=task.techdefence(2),
    Leader=task.techdefence(3),
    -- Assister = task.markingFront("First"),
    -- Special  = task.markingFront("Second"),
    match    = "[L][S][A]"
  },
  name = "Test_TechDefence",
  applicable = {
  	exp = "a",    
  	a = true
  },
  attribute = "attack",
  timeout = 99999

}
