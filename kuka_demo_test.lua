require 'qtcore'
require 'qtgui'
require 'qtuitools'
require 'os'
require 'rttlib'


-- in ui folder call : OROCOS_TARGET=xenomai rosrun ocl rttlua kuka_demo_test.lua -style plastique

app = QApplication(select('#',...) + 1, {'lua', ...})

function Q(s) return QString.fromUtf8(s, #s) end

uifile = QFile.new(Q"kukademo.ui")
uifile:open(QIODevice.OpenModeFlag.ReadOnly)

uiloader = QUiLoader.new()
window = uiloader:load(uifile)
window:show()

tc = rtt.getTC()
dp = tc:getPeer('Deployer')
dp:runScript("../kuka_simple_demo/orocos_script/kuka_demo.ops")
--dp:loadComponent('name','OCL::LuaComponent')
--name = dp:getPeer('name')
--name:exec_str('function updateHook() print(rtt.getTime()) end')
--name:setPeriod(0.1)
--name:configure()
--name:start()
kukademo = dp:getPeer("KukaDemo")
ATI = dp:getPeer("ATI")
 
app:__addmethod("startJacobian()", function()
    print ("Launch Kuka Transpose")
    dp:runScript("../kuka_jacobian_demo/orocos_script/KukaJacobianDemoRTNET.ops")
    kukademo = dp:getPeer("KukaDemo")
end)

app:__addmethod("startModel()", function()
    print ("Launch Kuka Model Transpose")
    dp:runScript("../kuka_model_demo/orocos_script/kuka_model_demo.ops")
    kukademo = dp:getPeer("KukaDemo")
end)

app:__addmethod("ATIcalibration()", function()
    print ("Launch ATI Calibration")
    dp:runScript("../kuka_ATI_calibration_demo/orocos_script/kukaATICalibrationDemoRTNET.ops")
    ATI = dp:getPeer("ATI")
    kukademo = dp:getPeer("KukaDemo")
end)

app:__addmethod("getNameKukaDemo()", function()
    print(kukademo:getName())
end)

app:__addmethod("setCons()", function()
    print(kukademo:getName())
    local Xdes = window:findChild("Xdes")
    local Ydes = window:findChild("Ydes")
    local Zdes = window:findChild("Zdes")

    local Xdes_value = Xdes:text()
    local Ydes_value = Ydes:text()
    local Zdes_value = Zdes:text()
    local pos_des = rtt.Variable("array")
    pos_des:resize(3)
    pos_des:fromtab{Xdes_value:toDouble(), Ydes_value:toDouble(), Zdes_value:toDouble()}
    kukademo:setXcons(pos_des)
end)

app:__addmethod("Start()", function()
    --getPeriod
    local period = window:findChild("period")
    local period_value = period:text()
    --sendPeriod to component
    kukademo:setPeriod(period_value:toDouble())
    kukademo:start()
end)

app:__addmethod("Stop()", function()
    kukademo:stop()
end)

app:__addmethod("SetControlStrategy()", function()
    local controlmode = window:findChild("controlmode")
    local controlmode_value = controlmode:text()
    kukademo:friStop()
	print(controlmode_value:toUInt())
    kukademo:setControlStrategy(controlmode_value:toUInt())
end)

app:__addmethod("fricmd()", function()
    kukademo:friStart()
end)

app:__addmethod("frimon()", function()
    kukademo:friStop()
end)

app:__addmethod("setJntImp()", function()
    local stiffness = window:findChild("stiffness")
    local stiffness_value = stiffness:text()
    local damping = window:findChild("damping")
    local damping_value = damping:text()
    kukademo:setJointImpedance(stiffness_value:toDouble(),damping_value:toDouble())
end)

app:__addmethod("setKpKd()", function()
    local Kp = window:findChild("kp")
    local Kp_value = Kp:text()	
    local Kd = window:findChild("kd")
    local Kd_value = Kd:text()	
    kukademo:setGains(Kp_value:toDouble(),Kd_value:toDouble())
end)

app:__addmethod("setTool()", function()
    local tool = window:findChild("settool")
    local tool_value = tool:text()
    kukademo:setTool(tool_value:toUInt())
end)

jacobian_demo = window:findChild("Jacobian_button")
jacobian_demo:connect('2clicked()', app, '1startJacobian()' )

ATI_calibration_demo = window:findChild("Calibration_button")
ATI_calibration_demo:connect('2clicked()', app, '1ATIcalibration()')

model_demo = window:findChild("model_button")
model_demo:connect('2clicked()', app, '1startModel()' )

SetCons = window:findChild("SetCons_button")
SetCons:connect('2clicked()', app, '1setCons()')

Start = window:findChild("Start_button")
Start:connect('2clicked()', app, '1Start()')

Stop = window:findChild("Stop_button")
Stop:connect('2clicked()', app, '1Stop()')

SetControlStrategy = window:findChild("SetControlMode_button")
SetControlStrategy:connect('2clicked()', app, '1SetControlStrategy()')

FRI_cmd_mode = window:findChild("fri_cmd_button")
FRI_cmd_mode:connect('2clicked()', app, '1fricmd()' )

FRI_mon_mode = window:findChild("fri_mon_button")
FRI_mon_mode:connect('2clicked()', app, '1frimon()' )

SetJointImpedance = window:findChild("set_joint_impedance_button")
SetJointImpedance:connect('2clicked()', app, '1setJntImp()' )

SetKpKd = window:findChild("setkpkd_button")
SetKpKd:connect('2clicked()', app, '1setKpKd()' )

SetTool = window:findChild("set_tool_button")
SetTool:connect('2clicked()', app, '1setTool()' )

timer= QTimer(parent)
timer:connect('2timeout()', function() 
	local pos_cart = rtt.Variable("array")
    	pos_cart:resize(3)
	pos_cart = kukademo:getCartPos() -- to add in friexamples
	--pos_cart:fromtab{1.0,2.2,-2.3}
	Xee = window:findChild("Xee")
	Yee = window:findChild("Yee")
	Zee = window:findChild("Zee")
	Xee:setText(QString():setNum(pos_cart[0]))
	Yee:setText(QString():setNum(pos_cart[1]))
	Zee:setText(QString():setNum(pos_cart[2]))

	local ATIvalues = rtt.Variable("array")
	ATIvalues:resize(8)
	ATIvalues = ATI:getATIvalues()
	Fx = window:findChild("data_Fx")
	Fy = window:findChild("data_Fy")
	Fz = window:findChild("data_Fz")
	Tx = window:findChild("data_Tx")
	Ty = window:findChild("data_Ty")
	Tz = window:findChild("data_Tz")
	Fnorm = window:findChild("data_normF")
	Tnorm = window:findChild("data_normT")

	local EstValues = rtt.Variable("array")
	EstValues:resize(8)
	EstValues = ATI:getKukaEstValues()
	Est_Fx = window:findChild("data_Festx")
	Est_Fy = window:findChild("data_Festy")
	Est_Fz = window:findChild("data_Festz")
	Est_Tx = window:findChild("data_Testx")
	Est_Ty = window:findChild("data_Testy")
	Est_Tz = window:findChild("data_Testz")
	Est_Fnorm = window:findChild("data_normFest")
	Est_Tnorm = window:findChild("data_normTest")

	Fx:setText(QString():setNum(ATIvalues[0]))
	Fy:setText(QString():setNum(ATIvalues[1]))
	Fz:setText(QString():setNum(ATIvalues[2]))
	Tx:setText(QString():setNum(ATIvalues[3]))
	Ty:setText(QString():setNum(ATIvalues[4]))
	Tz:setText(QString():setNum(ATIvalues[5]))
	Fnorm:setText(QString():setNum(ATIvalues[6]))
	Tnorm:setText(QString():setNum(ATIvalues[7]))

	Est_Fx:setText(QString():setNum(EstValues[0]))
	Est_Fy:setText(QString():setNum(EstValues[1]))
	Est_Fz:setText(QString():setNum(EstValues[2]))
	Est_Tx:setText(QString():setNum(EstValues[3]))
	Est_Ty:setText(QString():setNum(EstValues[4]))
	Est_Tz:setText(QString():setNum(EstValues[5]))
	Est_Fnorm:setText(QString():setNum(EstValues[6]))
	Est_Tnorm:setText(QString():setNum(EstValues[7]))

	local jacobian_matrix = rtt.Variable("array")
	jacobian_matrix:resize(42)
	jacobian_matrix = kukademo:getJacobian()
	J11 = window:findChild("J11")
	J12 = window:findChild("J12")
	J13 = window:findChild("J13")
	J14 = window:findChild("J14")
	J15 = window:findChild("J15")
	J16 = window:findChild("J16")
	J17 = window:findChild("J17")

	J21 = window:findChild("J21")
	J22 = window:findChild("J22")
	J23 = window:findChild("J23")
	J24 = window:findChild("J24")
	J25 = window:findChild("J25")
	J26 = window:findChild("J26")
	J27 = window:findChild("J27")

	J31 = window:findChild("J31")
	J32 = window:findChild("J32")
	J33 = window:findChild("J33")
	J34 = window:findChild("J34")
	J35 = window:findChild("J35")
	J36 = window:findChild("J36")
	J37 = window:findChild("J37")

	J41 = window:findChild("J41")
	J42 = window:findChild("J42")
	J43 = window:findChild("J43")
	J44 = window:findChild("J44")
	J45 = window:findChild("J45")
	J46 = window:findChild("J46")
	J47 = window:findChild("J47")

	J51 = window:findChild("J51")
	J52 = window:findChild("J52")
	J53 = window:findChild("J53")
	J54 = window:findChild("J54")
	J55 = window:findChild("J55")
	J56 = window:findChild("J56")
	J57 = window:findChild("J57")

	J61 = window:findChild("J61")
	J62 = window:findChild("J62")
	J63 = window:findChild("J63")
	J64 = window:findChild("J64")
	J65 = window:findChild("J65")
	J66 = window:findChild("J66")
	J67 = window:findChild("J67")

	J11:setText(QString():setNum(jacobian_matrix[0]))
	J12:setText(QString():setNum(jacobian_matrix[1]))
	J13:setText(QString():setNum(jacobian_matrix[2]))
	J14:setText(QString():setNum(jacobian_matrix[3]))
	J15:setText(QString():setNum(jacobian_matrix[4]))
	J16:setText(QString():setNum(jacobian_matrix[5]))
	J17:setText(QString():setNum(jacobian_matrix[6]))

	J21:setText(QString():setNum(jacobian_matrix[7]))
	J22:setText(QString():setNum(jacobian_matrix[8]))
	J23:setText(QString():setNum(jacobian_matrix[9]))
	J24:setText(QString():setNum(jacobian_matrix[10]))
	J25:setText(QString():setNum(jacobian_matrix[11]))
	J26:setText(QString():setNum(jacobian_matrix[12]))
	J27:setText(QString():setNum(jacobian_matrix[13]))
	
	J31:setText(QString():setNum(jacobian_matrix[14]))
	J32:setText(QString():setNum(jacobian_matrix[15]))
	J33:setText(QString():setNum(jacobian_matrix[16]))
	J34:setText(QString():setNum(jacobian_matrix[17]))
	J35:setText(QString():setNum(jacobian_matrix[18]))
	J36:setText(QString():setNum(jacobian_matrix[19]))
	J37:setText(QString():setNum(jacobian_matrix[20]))

	J41:setText(QString():setNum(jacobian_matrix[21]))
	J42:setText(QString():setNum(jacobian_matrix[22]))
	J43:setText(QString():setNum(jacobian_matrix[23]))
	J44:setText(QString():setNum(jacobian_matrix[24]))
	J45:setText(QString():setNum(jacobian_matrix[25]))
	J46:setText(QString():setNum(jacobian_matrix[26]))
	J47:setText(QString():setNum(jacobian_matrix[27]))

	J51:setText(QString():setNum(jacobian_matrix[28]))
	J52:setText(QString():setNum(jacobian_matrix[29]))
	J53:setText(QString():setNum(jacobian_matrix[30]))
	J54:setText(QString():setNum(jacobian_matrix[31]))
	J55:setText(QString():setNum(jacobian_matrix[32]))
	J56:setText(QString():setNum(jacobian_matrix[33]))
	J57:setText(QString():setNum(jacobian_matrix[34]))

	J61:setText(QString():setNum(jacobian_matrix[35]))
	J62:setText(QString():setNum(jacobian_matrix[36]))
	J63:setText(QString():setNum(jacobian_matrix[37]))
	J64:setText(QString():setNum(jacobian_matrix[38]))
	J65:setText(QString():setNum(jacobian_matrix[39]))
	J66:setText(QString():setNum(jacobian_matrix[40]))
	J67:setText(QString():setNum(jacobian_matrix[41]))
end)
timer:start(100) --msec

app.exec()
