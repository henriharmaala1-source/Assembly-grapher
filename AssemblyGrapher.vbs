Set sh  = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
dir = Left(WScript.ScriptFullName, InStrRev(WScript.ScriptFullName, "\"))
script = dir & "gui_main.py"

' Build list of candidate pythonw.exe locations
local = sh.ExpandEnvironmentStrings("%LOCALAPPDATA%") & "\Programs\Python\"
Dim vers(5)
vers(0) = "Python313" : vers(1) = "Python312" : vers(2) = "Python311"
vers(3) = "Python310" : vers(4) = "Python39"  : vers(5) = "Python38"

Dim pyw : pyw = ""
Dim i
For i = 0 To 5
    Dim cand : cand = local & vers(i) & "\pythonw.exe"
    If fso.FileExists(cand) Then pyw = cand : Exit For
Next

If pyw <> "" Then
    ' Found pythonw.exe — launch with no window
    sh.Run """" & pyw & """ """ & script & """", 0, False
Else
    ' Fallback: py launcher (installed to C:\Windows by every Python installer)
    sh.Run "py """ & script & """", 0, False
End If
