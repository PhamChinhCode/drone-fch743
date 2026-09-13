<#
.SYNOPSIS
    Nap lai bang tham so tu file `set ten=gia_tri` qua console USART1.

.DESCRIPTION
    Gui tung dong mot va doi firmware tra loi, thay vi dan ca file vao terminal:
    dem nhan cua CLI chi 256 byte, dan mot luc 167 dong la mat dong.

    In ra gia tri THUC SU firmware nhan cho moi dong. Firmware tu kep min/max,
    nen so in ra co the khac so trong file — do la dau hieu can xem lai.

    Mac dinh KHONG ghi flash. Them -Save khi da xem ket qua va chac chan.

.EXAMPLE
    .\nap_tham_so.ps1 -File truoc_offboard_khac_mac_dinh.txt
    .\nap_tham_so.ps1 -File truoc_offboard_khac_mac_dinh.txt -Save
    .\nap_tham_so.ps1 -File hien_tai_toan_bo.txt -Port COM3 -Save
#>
param(
    [Parameter(Mandatory = $true)] [string] $File,
    [string] $Port = 'COM4',
    [int]    $Baud = 921600,
    [switch] $Save
)

$path = if ([IO.Path]::IsPathRooted($File)) { $File } else { Join-Path $PSScriptRoot $File }
if (-not (Test-Path $path)) { throw "Khong thay file: $path" }

$sp = New-Object System.IO.Ports.SerialPort($Port, $Baud, 'None', 8, 'One')
$sp.ReadBufferSize = 131072
$sp.Open()

try {
    # Tat luong debug truoc, neu khong no chen vao cau tra loi.
    $sp.Write("mode 0`r`n"); Start-Sleep -Milliseconds 1500
    $sp.DiscardInBuffer()

    $sent = 0; $errors = 0
    foreach ($line in Get-Content $path) {
        $cmd = $line.Trim()
        if (-not $cmd -or $cmd.StartsWith('#')) { continue }

        $sp.DiscardInBuffer()
        $sp.Write("$cmd`r`n")
        Start-Sleep -Milliseconds 200
        $reply = (($sp.ReadExisting() -split "`n") | Where-Object { $_.Trim() } | Select-Object -First 1)

        # -cmatch (phan biet hoa thuong) va neo o dau dong. `-match 'ERR'` khong
        # phan biet hoa thuong nen bat nham ten tham so: offboard_stick_ovERRide.
        if ($reply -cmatch '^\s*ERR') { $errors++ }
        '{0,-45} -> {1}' -f $cmd, $reply
        $sent++
    }

    ''
    "Da gui $sent dong, $errors dong bao loi."

    if ($Save) {
        if ($errors -gt 0) {
            'CO LOI — KHONG ghi flash. Xem lai cac dong ERR roi chay lai.'
        } else {
            $sp.DiscardInBuffer()
            $sp.Write("save`r`n"); Start-Sleep -Milliseconds 2500
            $sp.ReadExisting().Trim()
        }
    } else {
        'Chua ghi flash (thieu -Save). Gia tri moi chi nam trong RAM.'
    }
}
finally {
    $sp.Close()
}
