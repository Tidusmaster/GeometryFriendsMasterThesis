loop 5
{
	SetWorkingDir, C:\Users\Tidusmaster\Source\Workspaces\Arbeitsbereich\GFC\Code\GeometryFriendsAgents\GeometryFriendsFiles
	Run, GeometryFriends.exe -l 12 7
	WinWaitActive, Geometry Friends
	Sleep 1000
	Send {Enter down}{Enter up}
	Sleep 50000
	WinClose
	Sleep 1000
}