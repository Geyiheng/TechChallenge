function Break(task)
  
    execute = function(runner)
      return CBreak(runner,600,0)
    end
  
    matchPos = function()
      
      return ball.pos()
    end
    
    return execute, matchPos
  end
  
gSkillTable.CreateSkill{
    name = "Break",
    execute = function (self)
      print("This is in skill"..self.name)
    end
}