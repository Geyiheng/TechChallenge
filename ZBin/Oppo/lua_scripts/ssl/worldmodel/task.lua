module(..., package.seeall)

--~		Play中统一处理的参数（主要是开射门）
--~		1 ---> task, 2 ---> matchpos, 3---->kick, 4 ---->dir,
--~		5 ---->pre,  6 ---->kp,       7---->cp,   8 ---->flag

--------------------------------------------------------------------------------------------

------------------------------------- 跑位相关的skill ---------------------------------------

--~ p为要走的点, d默认为射门朝向, f为相关的flag
function goSimplePos(p, d, f)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end

	if f ~= nil then
		iflag = f
	else
		iflag = 0
	end

	local mexe, mpos = SimpleGoto{pos = p, dir = idir, flag = iflag}
	return {mexe, mpos}
end

--~ p为要走的点, d默认为射门朝向, a为加速度, f为相关的flag
function goCmuRush(p, d, a, f)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end

	local mexe, mpos = GoCmuRush{pos = p, dir = idir, acc = a, flag = f}
	return {mexe, mpos}
end

------------------------------------- 射门相关的skill ---------------------------------------

function testBreak()
	local mexe, mpos = Break{}
	return {mexe, mpos}
end

function chase()
	local mexe, mpos = ChaseKick{dir = dir.chase}
	return {mexe, mpos, kick.flat, dir.chase, pre.middle, kp.full(), cp.full(), flag.nothing}
end

function chaseNew()
	local mexe, mpos = ChaseKickV2{dir = dir.chase}
	return {mexe, mpos, kick.flat, dir.chase, pre.high, kp.full(), cp.full(), flag.nothing}
end

-- 强制打近角，只在定位球中使用
function chaseNear()
	local idir = ball.toFuncDir(ball.refAntiYPos(CGeoPoint:new_local(param.pitchLength/2.0, 40)))
	local mexe, mpos = ChaseKick{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.full(), cp.full(), flag.nothing}
end

-- 强制打远角，只在定位球中使用
function chaseFar()
	local idir = ball.toFuncDir(ball.refSyntYPos(CGeoPoint:new_local(param.pitchLength/2.0, 40))) -- 大场的远角和近角不对
	local mexe, mpos = ChaseKick{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.full(), cp.full(), flag.nothing}
end

function chaseDirectShoot()
	local mexe, mpos = ChaseKick{dir = dir.chase}
	return {mexe, mpos, kick.flat, dir.chase, pre.low, kp.full(), cp.full(), flag.force_kick}
end

function chasePenalty(d)
	local mexe, mpos = ChaseKick{dir = d, flag = flag.dribbling}
	return {mexe, mpos, kick.flat, d, pre.middle, kp.full(), cp.full(), flag.force_kick}
end

------------------------------------- 传球相关的skill ---------------------------------------

-- role为接球车,也可以为某一个点
function pass(role, power)
	local idir,ikp
	if type(role) == "string" then
		idir = ball.toPlayerHeadDir(role)
	elseif type(role) == "function" then
		idir = ball.toFuncDir(role)
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end
	if power == nil then
		ikp =  kp.toTarget(role)
	else
		ikp = kp.specified(power)
	end
	local mexe, mpos = ChaseKickV2{pos = ball.pos, dir = idir}
	return {mexe, mpos, kick.auto(role), idir, pre.middle, ikp, cp.toPlayer(role), flag.nothing}
end

function passToDir(dir, c)
	local mexe, mpos = ChaseKick{pos = ball.pos, dir = dir}
	return {mexe, mpos, kick.flat, dir, pre.middle, kp.specified(c), cp.full(), flag.nothing}
end

function passToPos(p, c)
	local idir = player.toPointDir(p)
	local ipower
	if type(c)== "number" and c ~= nil then
		ipower=kp.specified(c)
	else
		ipower=kp.toTargetNormalPlay(p)
	end
	local mexe, mpos = ChaseKickV2{pos = ball.pos, dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.high, ipower, cp.full(), flag.nothing}
end


function chipPass(p, c, f, anti)
	local idir
	local ipower
	if type(p) == "string" then
		idir = ball.toPlayerHeadDir(p)
	elseif type(p) == "function" then
		if f == nil or f == true then
			idir = p
		elseif anti == false then 
			idir = function (role)
				return (p() - player.pos(role)):dir()
			end
		else
			idir = ball.antiYDir(p)
		end
	else
		if anti == false then
			idir = function (role)
				if type(p) == "userdata" then
					return (p - player.pos(role)):dir()
				end
			end
		else
			idir = player.antiYDir(p)
		end
	end

	if c == nil then
		if type(p) == "string" then
			ipower = cp.toPlayer(p)
		elseif type(p) == "userdata" then
			ipower = cp.toTarget(ball.antiYPos(p))
		elseif type(p) == "function" then
			if f == false or f == nil then
				if anti == true then
					ipower = cp.toTarget(ball.antiYPos(p))
				else
					ipower = cp.toTarget(p())
				end
			end
		end
	elseif type(c) == "number" then
		ipower = cp.specified(c)
	else
		ipower = c
	end
	local mexe, mpos = ChaseKick{pos = ball.pos,dir = idir}
	return {mexe, mpos, kick.chip, idir, pre.middle, kp.specified(0), ipower, flag.nothing}
end

function flatPass(role, power)
	local ipower
	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end
	local mexe, mpos = ChaseKick{pos = ball.pos, dir = ball.toPlayerHeadDir(role)}
	return {mexe, mpos, kick.flat, ball.toPlayerHeadDir(role), pre.low, ipower, cp.full(), flag.nothing}
end

function compensatePass(role,power)
	local ipower
	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end
	local mexe, mpos = ChaseKick{ pos = ball.pos, dir = player.toPlayerHeadDir(role)}
	return {mexe, mpos, kick.flat, player.toPlayerHeadDir(role), pre.middle, ipower, cp.full(), flag.nothing}
end

---------------------------------------- TouchKick -----------------------------------------

------------------------------------- 射门相关的skill ---------------------------------------

function touch(p, c)
	local idir

	-- 传入是否需要补偿的标志位，默认为需要补偿
	if c == nil or c == true then
		if p ~= nil then
			idir = dir.compensate(p)
		else
			idir = dir.compensate(CGeoPoint:new_local(param.pitchLength / 2, 0))
		end
	elseif c == false then
		if p ~= nil then
			idir = dir.nocompensation(p)
		else
			idir = dir.nocompensation(CGeoPoint:new_local(param.pitchLength / 2, 0))
		end
	end

	-- 根据射门位置做射门方向的固定，保证射门的坚定性
	-- by zhyaic 2013.6.27
	-- 可以在此处做，也可以在

	local mexe, mpos = TouchKick{dir = idir}
	-- to test forcekick
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.touch(), cp.full(), flag.force_kick}
end

-- 强制打近角，只在定位球中使用
function touchNear()
	local idir = dir.compensate(ball.refAntiYPos(CGeoPoint:new_local(param.pitchLength/2.0,15)))
	local mexe, mpos = TouchKick{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.touch(), cp.full(), flag.nothing}
end

-- 强制打远角，只在定位球中使用
function touchFar()
	local idir = dir.compensate(ball.refSyntYPos(CGeoPoint:new_local(param.pitchLength/2.0,15)))
	local mexe, mpos = TouchKick{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.touch(), cp.full(), flag.nothing}
end

function touchChip(d)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end
	local mexe, mpos = TouchKick{dir = idir}
	return {mexe, mpos, kick.chip, idir, pre.middle, kp.touch(), cp.touch(), flag.force_kick}
end

function touchCBack(d)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end
	local mexe, mpos = TouchKick{dir = idir}
	return {mexe, mpos, kick.chip, idir, pre.middle, kp.touch(), cp.slight(), flag.force_kick}
end

---------------------------------------- ShootBall -----------------------------------------

------------------------------------- 射门相关的skill ---------------------------------------

function shoot(f)
	local idir
	local iflags=flag.nothing
	idir = dir.shoot()
	if f == "safe" then
		iflags=flag.accurately
	end
	local mexe, mpos = ShootBall{dir = idir, pre = pre.high, flag=iflags}
	return {mexe, mpos}
end

function shootV2(p)
	local idir
	if p ~= nil then
		idir = dir.compensate(p)
	else
		idir = dir.shoot()
	end
	local mexe, mpos = ShootBallV2{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.full(), cp.full(), flag.nothing}
end

function shootV2AtEnemy(dir)
	local mexe, mpos = ShootBallV2{dir = dir}
	return {mexe, mpos, kick.flat, dir, pre.middle, kp.full(), cp.full(), flag.nothing}
end

------------------------------------- 传球相关的skill ---------------------------------------

function shootV2Pass(p, c)
	local idir
	local ipower
	if p ~= nil then
		-- idir = dir.compensate(p)
		idir = player.toPointDir(p)
	else
		idir = dir.shoot()
	end

	if c == nil then
		ipower = kp.toTarget(p)
	elseif type(c) == "number" then
		ipower = kp.specified(c)
	end
	local mexe, mpos = ShootBallV2{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.middle, ipower, cp.full, flag.nothing}
end

---------------------------------------- WaitKick ------------------------------------------

function waitTouch(p1,p2,style)
	local idir
	if style == "far" then
		idir = dir.compensate(ball.refAntiYPos(CGeoPoint:new_local(param.pitchLength/2.0,25)))
	elseif style == "near" then
		idir = dir.compensate(ball.refSyntYPos(CGeoPoint:new_local(param.pitchLength/2.0,25)))
	else
		idir = dir.compensate(CGeoPoint:new_local(param.pitchLength/2.0,0))
	end

	local mexe,mpos = WaitKick{ pos = p1, dir = idir, angle = p2}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.touch(), cp.full(), flag.force_kick}
end

function markingTouch(p,p1,p2,string)
	local iflags=0
	if string=="horizal" then
		iflags=flag.accurately
	end
	if string=="vertical" then
		iflags=0
	end
	local mexe,mpos = MarkingTouch{ areaNum = p, dir = dir.shoot(), leftUpPos = p1, rightDownPos = p2,flag=iflags}
	return {mexe, mpos, kick.flat, dir.shoot(), pre.middle, kp.touch(), cp.full(), flag.force_kick}
end

function fakeShoot(p)
	local mexe, mpos = GetBall{dir = ball.toPointDir(p)}
	return {mexe, mpos, kick.flat, ball.toPointDir(p), pre.specified(7), kp.specified(350), cp.full, flag.nothing}
end

function dribbleTurnShoot()
	local mexe, mpos = DribbleTurnKick{fDir = dir.shoot(),rotV=3.5,kPower=kp.full()}
	return {mexe, mpos}
end

function receive(p)
	local idir
	if type(p) == "string" then
		idir = player.toPlayerHeadDir(p)
	elseif type(p) == "function" then
		idir = p
	else
		idir = player.toPointDir(p)
	end

	local mexe, mpos = ReceivePass{dir = idir}
	return {mexe, mpos}
end

----------------------------------------------------------------------------------------------

------------------------------------ 传球相关的skill ---------------------------------------
function receivePass(p,c)
	local idir
	if type(p) == "string" then
		idir = player.toPlayerHeadDir(p)
	elseif type(p) == "function" then
		idir = p
	else
		idir = player.toPointDir(p)
	end

	local ipower
	if c == nil then
		ipower = kp.toTarget(p)
	else
		ipower = kp.specified(c)
	end
	local mexe, mpos = ReceivePass{dir = idir}
	return {mexe, mpos, kick.flat, idir, pre.high, ipower, cp.full(), flag.nothing}
end

function receiveChip(d, c)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end


	local mexe, mpos = ReceivePass{dir = idir}
	return {mexe, mpos, kick.chip, idir, pre.specified(3.5), kp.full(), cp.middle(), flag.nothing}
end

function receiveShoot()
	local mexe, mpos = ReceivePass{dir = dir.shoot()}
	return {mexe, mpos, kick.flat, dir.shoot(), pre.high, kp.full(), cp.full(), flag.nothing}
end

-- role 为接球车/接球点
function goAndTurnKick(role, power, icircle, a) -- 2014-03-28 added by yys, 增加转动圈数参数,并且第一个参数可以直接传入弧度
	local idir
	local ipower
	if type(role) == "number" then  -- 这里idir必须为函数，因为play.lua里面函数调用
		idir = function()
		  return role
		end
	elseif type(role) == "string" then
		idir = ball.toPlayerHeadDir(role)
	elseif type(role) == "function" then
		idir = role()
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end

	if power == nil then
		ipower = kp.specified(400)
		-- ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end

	if icircle == nil then
		icircle = 0
	end

	local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = idir,power = ipower, pre = pre.middle, circle = icircle, acc = a}
	return {mexe, mpos, kick.flat, idir, pre.high, ipower, cp.full, flag.nothing}
end

function goAndTurnKickByGetBall(role, power, icircle, a)
	local idir
	local ipower
	if type(role) == "number" then  -- 这里idir必须为函数，因为play.lua里面函数调用
		idir = function()
		  return role
		end
	elseif type(role) == "string" then
		idir = ball.toPlayerHeadDir(role)
	elseif type(role) == "function" then
		idir = ball.toFuncDir(role)
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end

	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end

	if icircle == nil then
		icircle = 0
	end

	local mexe, mpos = GoAndTurnKickV4{ pos = ball.pos, dir = idir, acc = a}
	return {mexe, mpos, kick.flat, idir, pre.low, ipower, cp.full, flag.nothing}
end

-- role 为接球车/接球点
function goAndTurnKickWithoutAnti(role, power, icircle, a)
	local idir
	local ipower
	if type(role) == "userdata" then
		idir = player.toPointDir(role)
	end

	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end

	if icircle == nil then
		icircle = 0
	end

	local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = idir, pre = pre.high, circle = icircle, acc = a}
	return {mexe, mpos, kick.flat, idir, pre.high, ipower, cp.full, flag.nothing}
end

-- role 为接球车/接球点
function goAndTurnChip(role, power, icircle, a) -- 2014-03-28 added by yys, 增加转动圈数参数,并且第一个参数可以直接传入弧度
	local idir
	local ipower
	if type(role) == "number" then  -- 这里idir必须为函数，因为play.lua里面函数调用
		idir = function()
		  return role
		end
	elseif type(role) == "string" then
		idir = ball.toPlayerHeadDir(role)
	elseif type(role) == "function" then
		idir = ball.toFuncDir(role)
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end

	if power == nil then
		ipower = cp.toTarget(role)
	else
		ipower = cp.specified(power)
	end

	if icircle == nil then
		icircle = 0
	end

	local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = idir, pre = pre.high, circle = icircle, acc = a}
	return {mexe, mpos, kick.chip, idir, pre.middle, kp.full, ipower, flag.nothing}
end

function goAndTurnKickOld(role, power)
	local idir
	local ipower
	if type(role) == "string" then
		idir = ball.toPlayerHeadDir(role)
	elseif type(role) == "function" then
		idir = ball.toFuncDir(role)
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end

	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end

	-- local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = idir, pre = pre.high, circle = 0}
	local mexe, mpos = GoAndTurnKick{ pos = ball.pos, dir = idir, pre = pre.high}
	return {mexe, mpos, kick.flat, idir, pre.high, ipower, cp.full, flag.nothing}
end


function goAndTurnKickQuick(role, power, ktype) -- 2015/04/23 at JP_Open by.YuN
	local idir
	local ipower
	if type(role) == "number" then  -- 这里idir必须为函数，因为play.lua里面函数调用
		idir = function()
		  return role
		end

	elseif type(role) == "string" then
		idir = ball.toPlayerHeadDir(role)
	elseif type(role) == "function" then
		idir = ball.toFuncDir(role)
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end

	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end

	if type(ipower)=="function" then
		ipower=ipower()
	end

	itype=0
	if type(ktype)=="string" then
		if ktype=="flat" then
			itype=0
		elseif ktype=="chip" then
			itype=1
		end
	end

	local mexe, mpos = GoAndTurnKick{ pos = ball.pos, dir = idir, type=itype, power=ipower}
	return {mexe, mpos}
end


function dribbleTurn(finalDir, adjustPre)
	local idir
	if type(finalDir) == "number" then
		idir = finalDir
	elseif type(finalDir) == "function" then
		idir = finalDir()
	end

	local ipre
	if adjustPre == nil then
		ipre = math.pi / 90
	else
		ipre = adjustPre
	end
	local mexe, mpos = DribbleTurn{ fDir = idir, pre = ipre }
	return {mexe, mpos}
end

function circlePass(role, power, c)
	local idir
	local ipower
	if type(role) == "string" then
		idir = player.toPlayerDir(role)
	elseif type(role) == "function" then
		idir = ball.toFuncDir(role)
	elseif type(role) == "userdata" then
		idir = player.antiYDir(role)
	end

	if power == nil then
		ipower = kp.toTarget(role)
	else
		ipower = kp.specified(power)
	end

	if c == nil then
		c = 2
	end
	-- local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = idir, pre = pre.high}
	local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = idir, pre = pre.high, circle = c}
	return {mexe, mpos, kick.flat, idir, pre.high, ipower, cp.full, flag.nothing}
end

function goAndTurn()
	local mexe, mpos = GoAndTurn{}
	return {mexe, mpos}
end

----------------------------------------------------------------------------------------------

------------------------------------ 跑位相关的skill ---------------------------------------
function goLeftSupport()
	-- local mexe, mpos = GoCmuRush{acc = 500, pos=pos.supportLeftPos2022, dir = player.toBallDir}
	local mexe, mpos = GoSupport{direction=1}
	return {mexe, mpos}
end

function goRightSupport()
	-- local mexe, mpos = GoCmuRush{acc = 500, pos=pos.supportRightPos2022, dir = player.toBallDir}
	local mexe, mpos = GoSupport{direction=0}
	return {mexe, mpos}
end

-- p为最终要踢向的点
function fakeStand(p)
	local mexe, mpos = GoCmuRush{pos = pos.fakeDown(p), dir = dir.fakeDown(p), flag = flag.slowly}
	return {mexe, mpos}
end

--~ p为要走的点,d默认为射门朝向
function goSpeciPos(p, d, f, a) -- 2014-03-26 增加a(加速度参数)
	local idir
	local iflag
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end

	if f ~= nil then
		iflag = f
	else
		iflag = 0
	end

	local mexe, mpos = GoCmuRush{pos = p, dir = idir, flag = iflag, acc = a}
	return {mexe, mpos}
end

-- role为传球车
function goBestShootPos(role)
	local mexe, mpos = GoCmuRush{ pos = pos.bestShoot, dir = player.toShootOrRobot(role)}
	return {mexe, mpos}
end

-- role为传球车
function goPassPos(role, f)
	local mexe, mpos = GoCmuRush{ pos = player.passPos(role), dir = player.toShootOrRobot(role), flag = f}
	return {mexe, mpos}
end

function goFirstPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.LWPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goSecondPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.RWPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goLWPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.LWPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goMWPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.MWPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goRWPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.RWPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goLMPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.LMPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goMMPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.MMPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goRMPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = ball.RMPassPos(), dir = player.toShootOrRobot(role),sender=role}
	return {mexe, mpos}
end

function goWaitAdvancePos(role)
	local mexe, mpos = GoCmuRush{ pos = player.waitAdvancePos(role), dir = dir.shoot()}
	return {mexe, mpos}
end

function goTechChalPos(role)
	local tmp = 0
	if role == "Pass" then
		tmp = 1
	end
	
	local mexe, mpos = GoTechChalPos{ pos = tmp }

	return {mexe, mpos}
end

function runMultiPos(p, c, d, idir, a)
	if c == nil then
		c = false
	end

	if d == nil then
		d = 20
	end

	if idir == nil then
		idir = dir.shoot()
	end

	local mexe, mpos = RunMultiPos{ pos = p, close = c, dir = idir, flag = flag.not_avoid_our_vehicle, dist = d, acc = a}
	return {mexe, mpos}
end

function runMultiPosVel(p, v, c, d, idir, a)
	if c == nil then
		c = false
	end

	if d == nil then
		d = 20
	end

	if idir == nil then
		idir = dir.shoot()
	end

	local mexe, mpos = RunMultiPosVel{ pos = p, vel = v, close = c, dir = idir, flag = flag.not_avoid_our_vehicle, dist = d, acc = a}
	return {mexe, mpos}
end


function gotoMatchPos(p, d)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end

	local mexe, mpos = GotoMatchPos{pos = p, dir = idir}
	return {mexe, mpos}
end

function goRecvPos(p, role)
	local mexe, mpos = GoCmuRush{ pos = p, dir = player.toPlayerDir(role)}
	return {mexe, mpos}
end

function goCmuRushVel(p, d, a, f, v)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end

	local mexe, mpos = GoCmuRush{pos = p, dir = idir, acc = a, flag = f, vel = v}
	return {mexe, mpos}
end

function goTestEmptyKick(p, d, a)
	local idir
	if d ~= nil then
		idir = d
	else
		idir = dir.shoot()
	end

	local mexe, mpos = GoCmuRush{pos = p, dir = idir, acc = a}
	return {mexe, mpos, kick.flat, idir, pre.middle, kp.full(),cp.full(), flag.force_kick}
end

function goTouchPos(p, t)
	local mexe, mpos = GoCmuRush{pos = p, dir = dir.evaluateTouch(t)}
	return {mexe, mpos}
end

----------------------------------------------------------------------------------------------

------------------------------------ 防守相关的skill ------------------------------------------

function protectBall()
	local mexe, mpos = ProtectBall{pos = pos.protectBall, flag = flag.dodge_ball}
	return {mexe, mpos}
end

function defendDefault(p)
	local mexe, mpos = GotoMatchPos{ pos = pos.defaultDefPos(p), dir = player.toBallDir, acc = 1200,flag = flag.dodge_referee_area}
	return {mexe, mpos}
end

function defendFront(p)
	local mexe, mpos = GotoMatchPos{ pos = pos.defendFrontPos(p), dir = player.toBallDir, acc = 1200}
	return {mexe, mpos}
end

function defendHead()
	local mexe, mpos = GotoMatchPos{ pos = pos.defendHeadPos, dir = player.toBallDir, acc = 800,flag = flag.not_avoid_their_vehicle}
	return {mexe, mpos}
end

function defendMiddleHead()
	local mexe, mpos = GotoMatchPos{ pos = pos.defendMiddleHeadPos, dir = player.toBallDir, acc = 800,flag = flag.not_avoid_their_vehicle}
	return {mexe, mpos}
end

function defendDribble()
	local mexe, mpos = GotoMatchPos{ pos = pos.defendDribblePos, dir = player.toBallDir, acc = 800,flag = flag.not_avoid_their_vehicle}
	return {mexe, mpos}
end

function defendKick(p1,p2,p3,p4)
	local mexe, mpos = GoCmuRush{ acc = 800, pos = pos.defendKickPos(p1,p2,p3,p4), dir = player.toBallDir}
	return {mexe, mpos}
end

function defendKickDir(p1,p2,p3,p4)
	local mexe, mpos = GoCmuRush{ acc = 800, pos = pos.defendKickPos(p1,p2,p3,p4), dir = player.toTheirGoal}
	return {mexe, mpos}
end

function defendOneKick(p1,p2,p3,p4,p5,p6)
	local mexe,mpos =  GoCmuRush{ acc = 800, pos = pos.oneKickDefPos(p1,p2,p3,p4,p5,p6), dir = player.toBallDir}
	return {mexe, mpos}
end

function defendTwoKick(p1,p2,p3,p4,p5,p6)
	local mexe,mpos =  GoCmuRush{ acc = 800, pos = pos.anotherKickDefPos(p1,p2,p3,p4,p5,p6), dir = player.toBallDir}
	return {mexe, mpos}
end

function leftBack(p)
	local ipower
	if p == nil then
		ipower = 2700
	else
		ipower = p
	end
	local mexe, mpos = GotoMatchPos{ method = 4, acc = 500, pos = pos.leftBackPos, dir = dir.backSmartGotoDir, srole = "leftBack", flag = bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle, flag.allow_dss)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
end

function rightBack(p)
	local ipower
	if p == nil then
		ipower = 2700
	else
		ipower = p
	end
	local mexe, mpos = GotoMatchPos{ method = 4, acc = 500, pos = pos.rightBackPos, dir = dir.backSmartGotoDir, srole = "rightBack", flag = bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle, flag.allow_dss)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.specified(ipower),cp.specified(ipower), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
end

function leftCenterBack()
	local mexe, mpos = GotoMatchPos{ method = 1, acc = 1000, pos = pos.leftCenterBackPos, dir = dir.backSmartGotoDir, srole = "leftCenterBack", flag = bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
end

function rightCenterBack()
	local mexe, mpos = GotoMatchPos{ method = 1, acc = 1000, pos = pos.rightCenterBackPos, dir = dir.backSmartGotoDir, srole = "rightCenterBack", flag = bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
end

function multiBack(guardNum, index, p)
	local ipower
	if p == nil then
		ipower = 450
	else
		ipower = p
	end
	local mexe, mpos = GotoMatchPos{ method = 4, sender = index, srole = "multiBack", acc = 1000, pos = pos.multiBackPos(guardNum, index), dir = dir.backSmartGotoDir, flag = bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.specified(ipower),cp.specified(ipower), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle)}
end

function leftBack4Stop()
	local STOP_FLAG = flag.dodge_ball
	local STOP_NO_DODGE_SELF = bit:_or(STOP_FLAG, flag.not_avoid_our_vehicle)
	local STOP_DSS = bit:_or(STOP_NO_DODGE_SELF, flag.allow_dss)
	local mexe, mpos = GotoMatchPos{ pos = pos.leftBackPos, dir = dir.backSmartGotoDir, flag = STOP_DSS}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), STOP_DSS}
end

function rightBack4Stop()
	local STOP_FLAG = flag.dodge_ball
	local STOP_NO_DODGE_SELF = bit:_or(STOP_FLAG, flag.not_avoid_our_vehicle)
	local STOP_DSS = bit:_or(STOP_NO_DODGE_SELF, flag.allow_dss)
	local mexe, mpos = GotoMatchPos{ pos = pos.rightBackPos, dir = dir.backSmartGotoDir, flag = STOP_DSS}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), STOP_DSS}
end

function singleBack4Stop()
	local STOP_FLAG = flag.dodge_ball
	local STOP_DSS = bit:_or(STOP_FLAG, flag.allow_dss)
	local mexe, mpos = GotoMatchPos{ pos = pos.singleBackPos, dir = dir.backSmartGotoDir, flag = STOP_DSS}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), STOP_DSS}
end

-- 此处的车为射门的车，如果不传则不避射门线
function defendMiddle(role, f)
	local mflag = f or 0
	if role == nil then
		mflag = 0--flag.not_avoid_our_vehicle
	else
		mflag = mflag--bit:_or(flag.avoid_shoot_line, flag.not_avoid_our_vehicle)
	end
	mflag=bit:_or(flag.allow_dss,mflag)
	local mexe, mpos = GotoMatchPos{ pos = pos.defendMiddlePos,dir = dir.backSmartGotoDir , srole = "defendMiddle", flag = mflag, sender = role}
	return {mexe, mpos, kick.flat, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), mflag}
end

function defendMiddle4Stop(role)
	local mflag
	if role == nil then
		mflag = 0--flag.not_avoid_our_vehicle
	else
		mflag = mflag--bit:_or(flag.avoid_shoot_line, flag.not_avoid_our_vehicle)
	end
	local STOP_FLAG = bit:_or(flag.dodge_ball)
	local STOP_DSS = bit:_or(STOP_FLAG, flag.allow_dss)
	mflag=bit:_or(STOP_DSS,mflag)
	local mexe, mpos = GotoMatchPos{ pos = pos.defendMiddlePos,dir = dir.backSmartGotoDir , srole = "defendMiddle", flag = mflag, sender = role}
	return {mexe, mpos, kick.flat, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), mflag}
end

function singleBack()
	local mexe, mpos = GotoMatchPos{ method = 1, acc = 1000, pos = pos.singleBackPos, dir = dir.backSmartGotoDir, srole = "singleBack",bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle, flag.allow_dss)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle, flag.allow_dss)}
end

function sideBack()
	local mexe, mpos = GotoMatchPos{ pos = pos.sideBackPos, dir = dir.sideBackDir, srole = "sideBack",bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle, flag.allow_dss)}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), bit:_or(flag.not_avoid_our_vehicle,flag.not_avoid_their_vehicle, flag.allow_dss)}
end

function sideBack4Stop()
	local STOP_FLAG = flag.dodge_ball
	local STOP_DSS = bit:_or(STOP_FLAG, flag.allow_dss)
	local mexe, mpos = GotoMatchPos{ pos = pos.sideBackPos, dir = dir.sideBackDir, srole = "sideBack",flag = STOP_DSS}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), STOP_DSS}
end

function goalie()
	local mexe, mpos = Goalie2013{ flag = flag.not_avoid_our_vehicle}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), flag.not_avoid_our_vehicle}
end

function goalieNew()
	local mexe, mpos = Goalie2022{ flag = flag.not_avoid_our_vehicle}
	return {mexe, mpos, kick.chip, dir.defendBackClear(), pre.fieldDefender(), kp.full(),cp.full(), flag.not_avoid_our_vehicle}
end

function penaltyGoalie()
	local mexe, mpos = PenaltyGoalie{}
	return {mexe, mpos, kick.chip ,dir.shoot(), pre.fieldDefender(),  kp.full(), cp.full(),flag.not_avoid_their_vehicle}
end

-- 用来盯人的skill,其中p为优先级
function marking(p)
	local mexe, mpos = Marking{pri = p}
	return {mexe, mpos}
end

function  markingDir(p, p0)
	local mexe, mpos = Marking{pri = p,dir = p0}
	return {mexe, mpos}
end

-- 强行绕前盯人
function markingFront(p)
	local mexe, mpos = Marking{pri = p, front = true, dir = dir.shoot()}
	return {mexe, mpos}
end

----------------------------------------------------------------------------------------------

----------------------------------------- 其他动作 --------------------------------------------
function penaltykick(role)
	local itandem = nil
	if type(role) == "string" then
		itandem = role
	elseif type(role) == "function" then
		itandem = role()
	end
	local mexe, mpos = PenaltyKickV2{pos = pos.advance, srole = "advancer", tandem = itandem}
	return {mexe, mpos}
end

function penaltykickshort(role)
	local itandem = nil
	if type(role) == "string" then
		itandem = role
	elseif type(role) == "function" then
		itandem = role()
	end
	local mexe, mpos = PenaltyKick{pos = pos.advance, srole = "advancer", tandem = itandem}
	return {mexe, mpos}
end


function advance(role)
	local itandem = nil
--	print(pos.advance)
	if type(role) == "string" then
		itandem = role
	elseif type(role) == "function" then
		itandem = role()
	end
	local mexe, mpos = AdvanceBall{pos = pos.advance, srole = "advancer", tandem = itandem}

	return {mexe, mpos}
	-- local mexe, mpos = Stop()
	-- return {mexe, mpos}
end

function advanceV4(role)
	local itandem = nil
	if type(role) == "string" then
		itandem = role
	elseif type(role) == "function" then
		itandem = role()
	end
	local mexe, mpos = AdvanceBallV4{pos = pos.advance, srole = "advancer", tandem = itandem}
	return {mexe, mpos}
end

function advanceV3(role)
	local itandem = nil
	if type(role) == "string" then
		itandem = role
	elseif type(role) == "function" then
		itandem = role()
	end
	local mexe, mpos = AdvanceBallV3{pos = pos.advance, srole = "advancer", tandem = itandem}
	return {mexe, mpos}
end

function getBall(p)
	-- local mexe, mpos = GetBall{ pos = pos.backBall(p), dir = dir.backBall(p)}
	local mexe, mpos = GetBall{ pos = ball.backPos(p), dir = ball.backDir(p)}
	return {mexe, mpos}
end

-- 注意，此处的点是用来做匹配的
-- 最终的StaticGetBall中调用的是拿球，离球4cm
-- anti若为false,则不将点进行反向
-- f为传入的flag
function staticGetBall(p, anti, f)
	-- local mexe, mpos = StaticGetBall{ pos = pos.backBall(p), dir = dir.backBall(p)}
	local mexe, mpos = StaticGetBall{ pos = ball.backPos(p, _, _, anti), dir = ball.backDir(p, anti), flag = f}
	return {mexe, mpos}
end

-- p为朝向，如果p传的是pos的话，不需要根据ball.antiY()进行反算
function goBackBall(p, d)
	local mexe, mpos = GoCmuRush{ pos = ball.backPos(p, d, 0), dir = ball.backDir(p), flag = flag.dodge_ball}
	return {mexe, mpos}
end

function goSupportPos(role)
	local mexe, mpos = GoCmuRush{pos = pos.getSupportPos(role), dir = dir.shoot(),sender=role}
	return {mexe, mpos}
end

function stop()
	local mexe, mpos = Stop()
	return {mexe, mpos}
end

-- f为false代表p为函数,并且返回值为userdata,p不能有参数
-- f为nil是为了兼容以前的情况
function slowGetBall(p, f, anti)
	local idir
	if type(p) == "string" then
		idir = player.toPlayerDir(p)
	elseif type(p) == "function" then
		if f == nil or f == true then
			idir = p
		else
			idir = ball.antiYDir(p)
		end
	else
		if anti == false then
			idir = function (role)
				if type(p) == "userdata" then
					return (p - player.pos(role)):dir()
				end
			end
		else
			idir = ball.antiYDir(p)
		end
	end

	local mexe, mpos = SlowGetBall{ dir = idir}
	return {mexe, mpos}
end

function getBallWithDir(dir)
	local idir = dir

	local mexe, mpos = SlowGetBall{ dir = idir}
	return {mexe, mpos}
end

function fetchBall(pos,power)
	local ikp
	local idir
	if type(pos) == "string" then
		idir = player.toPlayerHeadDir(pos)
	elseif type(pos) == "function" then
		idir = pos
	else
		idir = player.toPointDir(pos)
	end

	if power == nil then
		ikp =  kp.toTarget(role)
	else
		ikp = kp.specified(power)
	end
    
	local mexe, mpos = FetchBall{target = pos, kPower = ikp, dir = idir }
	return{mexe, mpos}
end

function slowGetBallForKick(d)
	local mexe, mpos = SlowGetBall{ dir = d}
	return {mexe, mpos}
end

function goAroundRobot(role, r)
	local tmpPos = function ()
		return player.pos(role)
	end
	local mexe, mpos = GoAroundRobot{ pos = tmpPos, dir = dir.shoot(), radius = r}
	return {mexe, mpos}
end

function goAroundBall(r)
	local mexe, mpos = GoAroundBall{ pos = ball.pos(), dir = dir.shoot(), radius = r}
	return {mexe, mpos}
end

function continue()
	return {["name"] = "continue"}
end
----------------------------------------------------------------------------------------------

------------------------------------ 测试相关的skill ------------------------------------------
function testChip()
	local mexe, mpos = ChaseKick{dir = dir.shoot(),flag = flag.dribbling}
	return {mexe, mpos, kick.chip, dir.shoot(), pre.specified(9), kp.full(), cp.full(), flag.dribbling}
end

function TestGoSpeciPos(p,d)
	local mexe, mpos = GoCmuRush{pos = p, dir = d}
	return {mexe, mpos}
end

function testCircleBall(role)
	local mexe, mpos
	if IS_SIMULATION then
		mexe, mpos = GoAndTurnKick{ pos = ball.pos, dir = player.toPlayerDir(role), pre = pre.low}
	else
		mexe, mpos = TestCircleBall{ pos = ball.pos, dir = player.toPlayerDir(role), pre = pre.low}
	end
	--local mexe, mpos = TestCircleBall{ pos = ball.pos, dir = player.toPlayerDir(role), pre = pre.low}
	return {mexe, mpos, kick.flat, player.toPlayerDir(role), pre.low, kp.specified(800), cp.toPlayer(role), flag.nothing}
end

function goAndTurnKickV3(role)
	local mexe, mpos = GoAndTurnKickV3{ pos = ball.pos, dir = player.toPlayerDir(role), pre = pre.middle}
	return {mexe, mpos, kick.flat, player.toPlayerDir(role), pre.middle, kp.specified(650), cp.toPlayer(role), flag.nothing}
end

function testGoPassPos(role)
	local mexe, mpos = GoCmuRush{ pos = player.testPassPos(role), dir = player.toPlayerDir(role)}
	return {mexe, mpos}
end

function testGoPassPosNew(role)
	local mexe, mpos = GoCmuRush{ pos = player.testPassPosNew(role), dir = player.toPlayerDir(role)}
	return {mexe, mpos}
end

function testGoPassPosThree(role1, role2)
	local mexe, mpos = GoCmuRush{ pos = player.testPassPosThree(role2), dir = player.toPlayerDir(role1)}
	return {mexe, mpos}
end

function testGoPassPosHard(role1, role2)
	local mexe, mpos = GoCmuRush{ pos = player.testPassPosHard(role2), dir = player.toPlayerDir(role1)}
	return {mexe, mpos}
end

function testSlowGetBall(p)
	local mexe, mpos = SlowGetBall{ dir = ball.antiYDir(p)}
	return {mexe, mpos}
end

function testKickOff1()
	local mexe,mpos = GoCmuRush{pos = pos.testTwoKickOffPos1(),dir=dir.playerToBall}
	return {mexe, mpos}
end

function testKickOff2()
	local mexe,mpos = GoCmuRush{pos = pos.testTwoKickOffPos2(),dir=dir.playerToBall}
	return {mexe, mpos}
end

function timeDelayTest()
	local mexe, mpos = TimeDelayTest{}
	return {mexe, mpos}
end