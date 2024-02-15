gPlayTable.CreatePlay{

	firstState = "start",

	["start"] = {
    switch = function ()
      if true then
        return "start"
      end
    end,
    Leader   = task.tech3pass(),
    Assister=task.tech3pass(),
    Special=task.tech3pass(),
    -- Assister = task.markingFront("First"),
    -- Special  = task.markingFront("Second"),
    match    = "[L][S][A]"
  },
  name = "Test_Tech3Pass",
  applicable = {
  	exp = "a",    
  	a = true
  },
  attribute = "attack",
  timeout = 99999

}