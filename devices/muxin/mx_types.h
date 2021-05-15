/*
 * File: mx_types.h
 * Project: mx_transform
 * File Created: Thursday, 30th July 2020 8:32:59 pm
 * Author: Else Zhan (else.zhan@mxi.ai)
 * -----
 * Last Modified: Thursday, 30th July 2020 10:23:29 pm
 * Modified By: Else Zhan (else.zhan@mxi.ai>)
 * -----
 * Copyright 2020 - 2020 mxi, mxi
 * -----
 * HISTORY:
 * Date      	By        	Comments
 * ----------	----------	---------------------------------------------------
 */

#ifndef __MX_TYPES_H__
#define __MX_TYPES_H__

#ifdef __cplusplus
extern "C" {
#endif


typedef struct point3f {
    float x;
    float y;
    float z;
}point3f_t;


typedef struct camera_parameters {
    float cx;
    float cy;
    float fx;
    float base_line;
}camera_parameters_t;

#ifdef __cplusplus
}
#endif
#endif
