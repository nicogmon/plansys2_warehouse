(define (problem warehouse_problem)
(:domain warehouse_domain)
    (:objects
        small_robot -robot
        medium_robot -robot
        big_robot -robot

        small_zone medium_zone inter_zone -zone
        s_central s_sh_1 s_sh_2 s_sh_3 -waypoint
        m_central m_sh_1 m_sh_2 m_sh_3 -waypoint
        warehouse_2_sh -waypoint
        common_zone -waypoint
        

        
        s_box_1 s_box_2 s_box_3 -box
        m_box_1 m_box_2 m_box_3 -box

    )
    
    (:init
        (robot_at small_robot unknown_point)
        (robot_zone small_robot small_zone)
        (idle_robot small_robot)
        (= (robot_capacity small_robot) 1)
        (= (current_robot_load small_robot) 0)

        (robot_at medium_robot m_sh_1)
        (robot_zone medium_robot medium_zone)
        (idle_robot medium_robot)
        (= (robot_capacity medium_robot) 3)
        (= (current_robot_load medium_robot) 0)

        (robot_at big_robot warehouse_2_sh)
        (robot_zone big_robot inter_zone)
        (idle_robot big_robot)
        (= (robot_capacity big_robot) 5)
        (= (current_robot_load big_robot) 0)


        (box_at s_box_1 s_sh_1)
        (box_at s_box_2 s_sh_2)
        (box_at s_box_3 s_sh_3)

        (box_at m_box_1 m_sh_1)
        (box_at m_box_2 m_sh_2)
        (box_at m_box_3 m_sh_3)   

        (waypoint_from_zone s_central small_zone)
        (waypoint_from_zone s_sh_1 small_zone)
        (waypoint_from_zone s_sh_2 small_zone)
        (waypoint_from_zone s_sh_3 small_zone)

        (waypoint_from_zone m_central medium_zone)
        (waypoint_from_zone m_sh_1 medium_zone)
        (waypoint_from_zone m_sh_2 medium_zone)
        (waypoint_from_zone m_sh_3 medium_zone)

        (waypoint_from_zone warehouse_2_sh inter_zone)

        (waypoint_from_zone common_zone small_zone)
        (waypoint_from_zone common_zone medium_zone)
        (waypoint_from_zone common_zone inter_zone)
        
        (waypoint_from_zone unknown_point small_zone)
        (waypoint_from_zone unknown_point medium_zone)
        (waypoint_from_zone unknown_point inter_zone)


        (connected common_zone s_sh_1)
        (connected common_zone s_sh_2)
        (connected common_zone s_sh_3)
        (connected common_zone s_central)
        (connected common_zone m_sh_1)
        (connected common_zone m_sh_2)
        (connected common_zone m_sh_3)
        (connected common_zone m_central)
        (connected common_zone warehouse_2_sh)
        (connected common_zone unknown_point)

        (= (distance_s common_zone s_sh_1 ) 110)
        (= (distance_s common_zone s_sh_2 ) 110)
        (= (distance_s common_zone s_sh_3 ) 120)
        (= (distance_s common_zone s_central ) 100)
        (= (distance_s common_zone m_sh_1 ) 55)
        (= (distance_s common_zone m_sh_2 ) 60)
        (= (distance_s common_zone m_sh_3 ) 52)
        (= (distance_s common_zone m_central ) 55)
        (= (distance_s common_zone warehouse_2_sh ) 80)

        (connected s_sh_1 common_zone)
        (connected s_sh_2 common_zone)
        (connected s_sh_3 common_zone)
        (connected s_central common_zone)
        (connected m_sh_1 common_zone)
        (connected m_sh_2 common_zone)
        (connected m_sh_3 common_zone)
        (connected m_central common_zone)
        (connected warehouse_2_sh common_zone)

        (= (distance_s s_sh_1 common_zone ) 110)
        (= (distance_s s_sh_2 common_zone ) 110)
        (= (distance_s s_sh_3 common_zone ) 120)
        (= (distance_s s_central common_zone ) 100)
        (= (distance_s m_sh_1 common_zone ) 55)
        (= (distance_s m_sh_2 common_zone ) 60)
        (= (distance_s m_sh_3 common_zone ) 52)
        (= (distance_s m_central common_zone ) 55)
        (= (distance_s warehouse_2_sh common_zone ) 80)

        (connected unknown_point s_sh_1)
        (connected unknown_point s_sh_2)
        (connected unknown_point s_sh_3)
        (connected unknown_point s_central)
        (connected unknown_point m_sh_1)
        (connected unknown_point m_sh_2)
        (connected unknown_point m_sh_3)
        (connected unknown_point m_central)
        (connected unknown_point warehouse_2_sh)
        (connected unknown_point common_zone)

        (= (distance_s unknown_point s_sh_1 ) 50)
        (= (distance_s unknown_point s_sh_2 ) 50)
        (= (distance_s unknown_point s_sh_3 ) 50)
        (= (distance_s unknown_point s_central ) 50)
        (= (distance_s unknown_point m_sh_1 ) 50)
        (= (distance_s unknown_point m_sh_2 ) 50)
        (= (distance_s unknown_point m_sh_3 ) 50)
        (= (distance_s unknown_point m_central ) 50)
        (= (distance_s unknown_point warehouse_2_sh ) 50)
        (= (distance_s unknown_point common_zone ) 50)



        (connected s_sh_1 s_sh_2)
        (connected s_sh_1 s_sh_3)
        (connected s_sh_1 s_central)
        (connected s_sh_2 s_sh_1)
        (connected s_sh_2 s_sh_3)
        (connected s_sh_2 s_central)
        (connected s_sh_3 s_sh_1)
        (connected s_sh_3 s_sh_2)
        (connected s_sh_3 s_central)
        (connected S_central s_sh_1)
        (connected S_central s_sh_2)
        (connected S_central s_sh_3)


        (= (distance_s s_sh_1 s_sh_2 ) 25)
        (= (distance_s s_sh_1 s_sh_3 ) 30)
        (= (distance_s s_sh_1 s_central ) 12)
        (= (distance_s s_sh_2 s_sh_1 ) 25)
        (= (distance_s s_sh_2 s_sh_3 ) 32)
        (= (distance_s s_sh_2 s_central ) 12)
        (= (distance_s s_sh_3 s_sh_1 ) 30)
        (= (distance_s s_sh_3 s_sh_2 ) 32)
        (= (distance_s s_sh_3 s_central ) 25)
        (= (distance_s s_central s_sh_1 ) 12)
        (= (distance_s s_central s_sh_2 ) 12)
        (= (distance_s s_central s_sh_3 ) 25)


        (connected m_sh_1 m_sh_2)
        (connected m_sh_1 m_sh_3)
        (connected m_sh_1 m_central)
        (connected m_sh_2 m_sh_1)
        (connected m_sh_2 m_sh_3)
        (connected m_sh_2 m_central)
        (connected m_sh_3 m_sh_1)
        (connected m_sh_3 m_sh_2)
        (connected m_sh_3 m_central)

        (= (distance_s m_sh_1 m_sh_2 ) 50)
        (= (distance_s m_sh_1 m_sh_3 ) 65)
        (= (distance_s m_sh_1 m_central ) 45)
        (= (distance_s m_sh_2 m_sh_1 ) 50)
        (= (distance_s m_sh_2 m_sh_3 ) 90)
        (= (distance_s m_sh_2 m_central ) 55)
        (= (distance_s m_sh_3 m_sh_1 ) 65)
        (= (distance_s m_sh_3 m_sh_2 ) 90)
        (= (distance_s m_sh_3 m_central ) 46)
        (= (distance_s m_central m_sh_1 ) 45)
        (= (distance_s m_central m_sh_2 ) 55)
        (= (distance_s m_central m_sh_3 ) 46)

        


    )
    
    (:goal
    (and
        ; (box_at s_box_1 s_sh_2)
        ; (box_at s_box_2 s_sh_3)
        (box_at s_box_1 warehouse_2_sh)
        (box_at s_box_2 warehouse_2_sh)
        (box_at s_box_3 warehouse_2_sh)
        (box_at m_box_1 warehouse_2_sh)
        (box_at m_box_2 warehouse_2_sh)
        (box_at m_box_3 warehouse_2_sh)
        ; (robot_at small_robot s_sh_2)
        ; (robot_at medium_robot m_sh_2)
        ; (robot_at big_robot common_zone)
    )
    )

;añadir accion y carga maxima de la zona comun.

    
    
) ; End Problem
