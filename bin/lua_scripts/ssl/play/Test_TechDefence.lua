gPlayTable.CreatePlay{

	firstState = "start",

	["start"] = {
    switch = function ()
      if true then
        return "start"
      end
    end,
    Kicker  = task.techdefence(),
    -- Assister = task.markingFront("First"),
    -- Special  = task.markingFront("Second"),
    match    = ""
  },
  name = "Test_TechDefence",
  applicable = {
  	exp = "a",    
  	a = true
  },
  attribute = "attack",
  timeout = 99999

}
-- [1,2,3]
-- [1,2,3,2,3]
-- [,2]
-- [,3]
-- [3]