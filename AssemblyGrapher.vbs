Set sh  = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
dir     = Left(WScript.ScriptFullName, InStrRev(WScript.ScriptFullName, "\"))
script  = dir & "gui_main.py"
user    = sh.ExpandEnvironmentStrings("%USERPROFILE%")
local   = sh.ExpandEnvironmentStrings("%LOCALAPPDATA%")

' ── Build candidate list (pythonw.exe paths, best first) ─────────────────────
' Conda / Miniforge / Mamba environments first — these are where OCC lives
Dim cands(19)
cands(0)  = user  & "\miniconda3\pythonw.exe"
cands(1)  = user  & "\miniconda3\envs\occ\pythonw.exe"
cands(2)  = user  & "\anaconda3\pythonw.exe"
cands(3)  = user  & "\anaconda3\envs\occ\pythonw.exe"
cands(4)  = user  & "\miniforge3\pythonw.exe"
cands(5)  = user  & "\mambaforge\pythonw.exe"
cands(6)  = local & "\miniconda3\pythonw.exe"
cands(7)  = local & "\anaconda3\pythonw.exe"
cands(8)  = local & "\miniforge3\pythonw.exe"
cands(9)  = local & "\mambaforge\pythonw.exe"
cands(10) = "C:\miniconda3\pythonw.exe"
cands(11) = "C:\anaconda3\pythonw.exe"
cands(12) = "C:\miniforge3\pythonw.exe"
cands(13) = "C:\ProgramData\miniconda3\pythonw.exe"
cands(14) = "C:\ProgramData\anaconda3\pythonw.exe"
' Standard Python installer (AppData) — fallback
cands(15) = local & "\Programs\Python\Python313\pythonw.exe"
cands(16) = local & "\Programs\Python\Python312\pythonw.exe"
cands(17) = local & "\Programs\Python\Python311\pythonw.exe"
cands(18) = local & "\Programs\Python\Python310\pythonw.exe"
cands(19) = local & "\Programs\Python\Python39\pythonw.exe"

' ── Pick first existing pythonw ───────────────────────────────────────────────
Dim chosen : chosen = ""
Dim i
For i = 0 To 19
    If fso.FileExists(cands(i)) Then
        chosen = cands(i)
        Exit For
    End If
Next

' ── Launch ────────────────────────────────────────────────────────────────────
If chosen <> "" Then
    sh.Run """" & chosen & """ """ & script & """", 0, False
Else
    ' Last resort: py launcher (always at C:\Windows\py.exe)
    sh.Run "py """ & script & """", 0, False
End If
