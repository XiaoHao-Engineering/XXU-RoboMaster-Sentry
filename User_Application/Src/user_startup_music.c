#include "../../Core/Inc/bsp.h"
#ifdef HAL_TIM_MODULE_ENABLED
/* 包含头文件 */
#include "../user_startup_music.h"
#include "../../User_Architect/user_systick.h"

/* 私有函数声明 */
static void StartupMusic_Update(void *arg);

/* 函数体 */

static void StartupMusic_Update(void *arg) {
    STARTUP_MUSIC_DRIVES *music_player = (STARTUP_MUSIC_DRIVES *)arg;

    if (music_player->state != MUSIC_PLAYING) {
        return;
    }

    if (music_player->current_note >= music_player->note_count && music_player->time_counter == 0) {
        Buzzer_Off(music_player->buzzer);
        music_player->state = MUSIC_FINISHED;
        SysTick_StopTask(music_player->task);
        return;
    }

    if (music_player->time_counter > 0) {
        music_player->time_counter--;
        return;
    }

    if (music_player->current_note < music_player->note_count) {
        const uint16_t frequency = music_player->music[music_player->current_note][0];
        const uint16_t duration = music_player->music[music_player->current_note][1];

        if (frequency == 0) {
            Buzzer_Off(music_player->buzzer);
        } else {
            Buzzer_On(music_player->buzzer, frequency);
        }

        music_player->time_counter = duration;
        music_player->current_note++;
    }
}

void StartupMusic_Init(STARTUP_MUSIC_DRIVES *music_player, BUZZER_DRIVES *buzzer, SysTick_Task *task,
                       const uint16_t (*music_data)[2], const uint16_t note_count) {

    music_player->buzzer = buzzer;
    music_player->music = music_data;
    music_player->task = task;
    music_player->note_count = note_count;
    music_player->current_note = 0;
    music_player->time_counter = 0;
    music_player->state = MUSIC_IDLE;

    SysTick_InitTask(task, (void *)music_player, 1, 1, Task_REPEAT, StartupMusic_Update);
    SysTick_StartTask(task);
}

void StartupMusic_Start(STARTUP_MUSIC_DRIVES *music_player) {
    music_player->current_note = 0;
    music_player->time_counter = 0;
    music_player->state = MUSIC_PLAYING;
}

#endif /* HAL_TIM_MODULE_ENABLED */
