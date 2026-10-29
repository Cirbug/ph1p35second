/*
 * Copyright (c) 2023, Anlogic Inc. and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <math.h>
#include <time.h>
#include "al_core.h"
#include <stdlib.h>

AL_S32 main()
{
    const AL_CHAR *str = "Hello World From Anlogic!";
    unsigned int i;

    if (AL_BSP_VERSION_PATCH == 0)
        AL_LOG(AL_LOG_LEVEL_NOTICE, "%s BSP version:%s.%d.%d\r\n", str, AL_BSP_NAME, AL_BSP_VERSION_MAJOR, AL_BSP_VERSION_MINOR);
    else
        AL_LOG(AL_LOG_LEVEL_NOTICE, "%s BSP version:%s.%d.%d.sp%d\r\n", str, AL_BSP_NAME,
                                                AL_BSP_VERSION_MAJOR, AL_BSP_VERSION_MINOR, AL_BSP_VERSION_PATCH);

    AL_LOG(AL_LOG_LEVEL_NOTICE, "    __  __     ____         _       ______       __    __ \r\n");
    AL_LOG(AL_LOG_LEVEL_NOTICE, "   / / / /_   / / /___     | |     / / __ \\_____/ ____/ / \r\n");
    AL_LOG(AL_LOG_LEVEL_NOTICE, "  / /_/ / _ \\/ / / __ \\    | | /| / / / / / ___/ / __  /  \r\n");
    AL_LOG(AL_LOG_LEVEL_NOTICE, " / __  /  __/ / / /_/ /    | |/ |/ / /_/ / /  / / /_/ /   \r\n");
    AL_LOG(AL_LOG_LEVEL_NOTICE, "/_/ /_/\\___/_/_/\\____/     |__/|__/\\____/_/  /_/\\__ _/    \r\n");

    return 0;
}
