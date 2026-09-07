$canSource = Get-Content -Raw "$PSScriptRoot\..\Hardware\can.c"

$required = @(
    '__HAL_RCC_GPIOA_CLK_ENABLE\(\)',
    'GPIO_PIN_11',
    'GPIO_PIN_12',
    'HAL_GPIO_Init\(GPIOA, &GPIO_InitStruct\)',
    'HAL_GPIO_DeInit\(GPIOA, GPIO_PIN_11\|GPIO_PIN_12\)'
)

foreach ($pattern in $required) {
    if ($canSource -notmatch $pattern) {
        Write-Error "CAN1 default PA11/PA12 mapping is missing: $pattern"
        exit 1
    }
}

if ($canSource -match '__HAL_AFIO_REMAP_CAN1_2\(\)' -or
    $canSource -match 'HAL_GPIO_Init\(GPIOB, &GPIO_InitStruct\)') {
    Write-Error 'CAN1 still uses PB8/PB9 remapping instead of default PA11/PA12'
    exit 1
}

Write-Output 'PASS: CAN1 uses external transceiver on default PA11/PA12 pins'
