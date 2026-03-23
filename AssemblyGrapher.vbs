Set sh  = CreateObject("WScript.Shell")
Set fso = CreateObject("Scripting.FileSystemObject")
dir     = Left(WScript.ScriptFullName, InStrRev(WScript.ScriptFullName, "\"))
script  = dir & "gui_main.py"
user    = sh.ExpandEnvironmentStrings("%USERPROFILE%")
local   = sh.ExpandEnvironmentStrings("%LOCALAPPDATA%")

Dim chosen : chosen = ""

' ── 1. Find conda.exe and ask it where its base environment lives ──────────
' conda.exe is always at <env_root>\Scripts\conda.exe
' We search miniforge3/mamba first (most modern), anaconda3 last (older)
Dim condaScripts(11)
condaScripts(0)  = user  & "\miniforge3\Scripts\conda.exe"
condaScripts(1)  = user  & "\mambaforge\Scripts\conda.exe"
condaScripts(2)  = user  & "\miniconda3\Scripts\conda.exe"
condaScripts(3)  = user  & "\anaconda3\Scripts\conda.exe"
condaScripts(4)  = local & "\miniforge3\Scripts\conda.exe"
condaScripts(5)  = local & "\mambaforge\Scripts\conda.exe"
condaScripts(6)  = local & "\miniconda3\Scripts\conda.exe"
condaScripts(7)  = local & "\anaconda3\Scripts\conda.exe"
condaScripts(8)  = "C:\miniforge3\Scripts\conda.exe"
condaScripts(9)  = "C:\miniconda3\Scripts\conda.exe"
condaScripts(10) = "C:\ProgramData\miniforge3\Scripts\conda.exe"
condaScripts(11) = "C:\ProgramData\miniconda3\Scripts\conda.exe"

Dim j
For j = 0 To 11
    If fso.FileExists(condaScripts(j)) Then
        On Error Resume Next
        Dim oExec
        Set oExec = sh.Exec("""" & condaScripts(j) & """ info --base")
        If Err.Number = 0 Then
            Dim condaBase : condaBase = ""
            Do While Not oExec.StdOut.AtEndOfStream
                Dim ln : ln = Trim(oExec.StdOut.ReadLine())
                If ln <> "" Then condaBase = ln
            Loop
            If condaBase <> "" Then
                If fso.FileExists(condaBase & "\pythonw.exe") Then
                    chosen = condaBase & "\pythonw.exe"
                ElseIf fso.FileExists(condaBase & "\python.exe") Then
                    chosen = condaBase & "\python.exe"
                End If
            End If
        End If
        On Error GoTo 0
        If chosen <> "" Then Exit For
    End If
Next

' ── 2. Static path scan fallback ──────────────────────────────────────────
If chosen = "" Then
    Dim cands(21)
    cands(0)  = user  & "\miniforge3\pythonw.exe"
    cands(1)  = user  & "\miniforge3\python.exe"
    cands(2)  = user  & "\mambaforge\pythonw.exe"
    cands(3)  = user  & "\mambaforge\python.exe"
    cands(4)  = user  & "\miniconda3\pythonw.exe"
    cands(5)  = user  & "\miniconda3\python.exe"
    cands(6)  = user  & "\anaconda3\pythonw.exe"
    cands(7)  = user  & "\anaconda3\python.exe"
    cands(8)  = local & "\miniforge3\pythonw.exe"
    cands(9)  = local & "\miniforge3\python.exe"
    cands(10) = local & "\miniconda3\pythonw.exe"
    cands(11) = local & "\miniconda3\python.exe"
    cands(12) = local & "\anaconda3\pythonw.exe"
    cands(13) = local & "\anaconda3\python.exe"
    cands(14) = "C:\miniforge3\pythonw.exe"
    cands(15) = "C:\miniconda3\pythonw.exe"
    cands(16) = "C:\ProgramData\miniforge3\pythonw.exe"
    cands(17) = "C:\ProgramData\miniconda3\pythonw.exe"
    cands(18) = local & "\Programs\Python\Python313\pythonw.exe"
    cands(19) = local & "\Programs\Python\Python312\pythonw.exe"
    cands(20) = local & "\Programs\Python\Python311\pythonw.exe"
    cands(21) = local & "\Programs\Python\Python310\pythonw.exe"

    Dim i
    For i = 0 To 21
        If fso.FileExists(cands(i)) Then
            chosen = cands(i)
            Exit For
        End If
    Next
End If

' ── 3. Launch ──────────────────────────────────────────────────────────────
If chosen <> "" Then
    sh.Run """" & chosen & """ """ & script & """", 0, False
Else
    sh.Run "py """ & script & """", 0, False
End If
