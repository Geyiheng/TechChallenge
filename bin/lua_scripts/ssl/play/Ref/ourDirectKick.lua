if OPPONENT_NAME == "NormalKick" then
	if ball.refPosX() > 360 and math.abs(ball.refPosY()) > 220 then
	    dofile("../lua_scripts/ssl/play/Ref/DirectCornerKick/DirectCornerKick.lua")
	elseif ball.refPosX() > 100 then
	    dofile("../lua_scripts/ssl/play/Ref/DirectFrontKick/DirectFrontKick.lua")
	elseif ball.refPosX()>-100 then
	    dofile("../lua_scripts/ssl/play/Ref/MiddleKick/MiddleKick.lua")
	else
	    dofile("../lua_scripts/ssl/play/Ref/BackKick/BackKick.lua")
	end
else
	if ball.refPosX() > 360 and math.abs(ball.refPosY()) > 220 then
		dofile("../lua_scripts/ssl/play/Ref/DirectCornerKick/DirectCornerKick.lua")
	elseif ball.refPosX()>0 then
		dofile("../lua_scripts/ssl/play/Ref/DirectFrontKick/DirectFrontKick.lua")
	else
		dofile("../lua_scripts/ssl/play/Ref/BackKick/BackKick.lua")
	end
end